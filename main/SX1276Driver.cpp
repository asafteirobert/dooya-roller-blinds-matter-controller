#include "SX1276Driver.hpp"
#include "Constants.hpp"

#include <algorithm>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace
{
// --- OOK/PWM timing
constexpr uint32_t SYNC_US = 4600;
constexpr uint32_t SHORT_US = 350; // logical bit 0
constexpr uint32_t LONG_US = 700;  // logical bit 1
constexpr uint32_t RESET_US = 7500; // silence between repeated frames

// On-air sample period. Must evenly divide every timing value above (it's the GCD: 50 = 2*5^2).
constexpr uint32_t SAMPLE_US = 50;
static_assert(SYNC_US % SAMPLE_US == 0 && SHORT_US % SAMPLE_US == 0 && LONG_US % SAMPLE_US == 0,
              "SAMPLE_US must evenly divide the pulse widths or the waveform will be built with rounding error");

constexpr uint32_t SYNC_CHIPS = SYNC_US / SAMPLE_US;
constexpr uint32_t SHORT_CHIPS = SHORT_US / SAMPLE_US;
constexpr uint32_t LONG_CHIPS = LONG_US / SAMPLE_US;

constexpr double FXOSC_HZ = 32000000.0;
constexpr double RF_FREQUENCY_HZ = 433915000.0; // Dooya remotes operate in the 433 MHz ISM band
constexpr uint32_t BIT_RATE_BPS = 1000000 / SAMPLE_US; // one on-air bit == one SAMPLE_US chip

constexpr int SPI_CLOCK_HZ = 4 * 1000 * 1000; // conservative for hobby-wired breakout boards

// The SX1276's FSK/OOK FIFO is only 64 bytes deep, so any payload longer than that (ours always
// is) must be streamed in while transmitting, per the "Handling Large Packets" procedure in the
// datasheet: pre-fill, enable Tx, then keep topping up in FIFO_CHUNK-sized bursts whenever the
// FifoLevel flag (mirrored on DIO1) says the level has dropped back to the threshold.
constexpr size_t FIFO_CAPACITY = 64;
constexpr size_t FIFO_CHUNK = 32;
constexpr uint8_t FIFO_THRESHOLD = FIFO_CAPACITY - FIFO_CHUNK; // leaves exactly one chunk of headroom

constexpr int64_t FIFO_WAIT_TIMEOUT_US = 500000; // generous vs. the ~60-90 ms a real transfer takes

// --- SX1276 FSK/OOK register map (see RFM9x/SX1276 datasheet section 6.2) ---
constexpr uint8_t REG_FIFO = 0x00;
constexpr uint8_t REG_OP_MODE = 0x01;
constexpr uint8_t REG_BITRATE_MSB = 0x02;
constexpr uint8_t REG_BITRATE_LSB = 0x03;
constexpr uint8_t REG_FRF_MSB = 0x06;
constexpr uint8_t REG_FRF_MID = 0x07;
constexpr uint8_t REG_FRF_LSB = 0x08;
constexpr uint8_t REG_PA_CONFIG = 0x09;
constexpr uint8_t REG_PA_RAMP = 0x0A;
constexpr uint8_t REG_PREAMBLE_MSB = 0x25;
constexpr uint8_t REG_PREAMBLE_LSB = 0x26;
constexpr uint8_t REG_SYNC_CONFIG = 0x27;
constexpr uint8_t REG_PACKET_CONFIG1 = 0x30;
constexpr uint8_t REG_PACKET_CONFIG2 = 0x31;
constexpr uint8_t REG_PAYLOAD_LENGTH = 0x32;
constexpr uint8_t REG_FIFO_THRESH = 0x35;
constexpr uint8_t REG_VERSION = 0x42;

constexpr uint8_t SPI_WRITE_BIT = 0x80;

// RegOpMode: LongRangeMode=0 (FSK/OOK) | ModulationType=01 (OOK) | LowFrequencyModeOn=1 (<525MHz)
constexpr uint8_t MODULATION_OOK_LF = 0x28;
constexpr uint8_t OPMODE_SLEEP = MODULATION_OOK_LF | 0x00;
constexpr uint8_t OPMODE_STANDBY = MODULATION_OOK_LF | 0x01;
constexpr uint8_t OPMODE_TX = MODULATION_OOK_LF | 0x03;

// Appends single-value-per-chip runs to a byte buffer, MSB-first, matching the bit order the
// SX1276 shifts a FIFO byte out in 
class BitWriter
{
public:
    explicit BitWriter(std::vector<uint8_t>& buffer) : buffer_(buffer) {}

    void appendRun(bool level, uint32_t chipCount)
    {
        for (uint32_t i = 0; i < chipCount; ++i)
        {
            size_t byteIndex = bitCount_ / 8;
            uint8_t bitIndex = 7 - static_cast<uint8_t>(bitCount_ % 8);
            if (byteIndex == buffer_.size())
            {
                buffer_.push_back(0);
            }
            if (level)
            {
                buffer_[byteIndex] |= static_cast<uint8_t>(1u << bitIndex);
            }
            ++bitCount_;
        }
    }

private:
    std::vector<uint8_t>& buffer_;
    size_t bitCount_ = 0;
};

// Encodes the 40-bit Dooya command into the literal on-air OOK waveform for a single frame
// (sync + 40 bits, each chip SAMPLE_US wide). See the header-file comment for the short/long/invert
// derivation.
std::vector<uint8_t> buildWaveform(const std::array<uint8_t, 5>& data)
{
    std::vector<uint8_t> waveform;
    BitWriter writer(waveform);

    writer.appendRun(true, SYNC_CHIPS);
    writer.appendRun(false, LONG_CHIPS);
    writer.appendRun(false, LONG_CHIPS);

    for (uint8_t byte : data)
    {
        for (int bitIndex = 7; bitIndex >= 0; --bitIndex)
        {
            bool bit = (byte >> bitIndex) & 0x01;
            // Mark and space swap between LONG/SHORT so every bit takes the same
            // SHORT_CHIPS+LONG_CHIPS total time regardless of its value.
            writer.appendRun(true, bit ? LONG_CHIPS : SHORT_CHIPS);
            writer.appendRun(false, !bit ? LONG_CHIPS : SHORT_CHIPS);
        }
    }

    return waveform;
}
} // namespace

void SX1276Driver::resetChip()
{
    gpio_config_t resetConfig = {};
    resetConfig.pin_bit_mask = 1ULL << SX1276_RESET_GPIO;
    resetConfig.mode = GPIO_MODE_OUTPUT;
    resetConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    resetConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    resetConfig.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&resetConfig);

    // Manual reset per datasheet section 7.2.2: NRESET low for >100us, then wait >=5ms.
    gpio_set_level(SX1276_RESET_GPIO, 0);
    esp_rom_delay_us(200);
    gpio_set_level(SX1276_RESET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void SX1276Driver::configureDioPins()
{
    // DIO0 = PacketSent, DIO1 = FifoLevel in Tx/packet mode -- both are the default DioMapping
    // (RegDioMapping1 reset value 0x00), so only the GPIO direction needs configuring here.
    gpio_config_t dioConfig = {};
    dioConfig.pin_bit_mask = (1ULL << SX1276_DIO0_GPIO) | (1ULL << SX1276_DIO1_GPIO);
    dioConfig.mode = GPIO_MODE_INPUT;
    dioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    dioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    dioConfig.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&dioConfig);
}

void SX1276Driver::writeRegister(uint8_t address, uint8_t value)
{
    uint8_t buffer[2] = { static_cast<uint8_t>(address | SPI_WRITE_BIT), value };
    spi_transaction_t transaction = {};
    transaction.length = 8 * sizeof(buffer);
    transaction.tx_buffer = buffer;
    spi_device_polling_transmit(spiDevice, &transaction);
}

uint8_t SX1276Driver::readRegister(uint8_t address)
{
    uint8_t tx[2] = { static_cast<uint8_t>(address & ~SPI_WRITE_BIT), 0x00 };
    uint8_t rx[2] = {};
    spi_transaction_t transaction = {};
    transaction.length = 8 * sizeof(tx);
    transaction.tx_buffer = tx;
    transaction.rx_buffer = rx;
    spi_device_polling_transmit(spiDevice, &transaction);
    return rx[1];
}

void SX1276Driver::writeFifo(const uint8_t* data, size_t length)
{
    uint8_t buffer[FIFO_CHUNK + 1];
    buffer[0] = REG_FIFO | SPI_WRITE_BIT;
    std::copy(data, data + length, buffer + 1);

    spi_transaction_t transaction = {};
    transaction.length = 8 * (length + 1);
    transaction.tx_buffer = buffer;
    spi_device_polling_transmit(spiDevice, &transaction);
}

void SX1276Driver::configureRadio()
{
    // LongRangeMode can only change while the *target* mode is Sleep, so force that first
    // regardless of whatever mode the chip happened to power up or warm-reset into.
    writeRegister(REG_OP_MODE, 0x00);
    writeRegister(REG_OP_MODE, OPMODE_SLEEP);
    writeRegister(REG_OP_MODE, OPMODE_STANDBY);
    vTaskDelay(pdMS_TO_TICKS(10)); // crystal oscillator startup

    uint32_t frf = static_cast<uint32_t>((RF_FREQUENCY_HZ * 524288.0) / FXOSC_HZ + 0.5);
    writeRegister(REG_FRF_MSB, static_cast<uint8_t>(frf >> 16));
    writeRegister(REG_FRF_MID, static_cast<uint8_t>(frf >> 8));
    writeRegister(REG_FRF_LSB, static_cast<uint8_t>(frf));

    // Bit rate sets the on-air duration of a single chip in our hand-encoded waveform (see
    // buildWaveform); it has nothing to do with any "real" data rate.
    uint32_t bitRateReg = static_cast<uint32_t>(FXOSC_HZ / BIT_RATE_BPS + 0.5);
    writeRegister(REG_BITRATE_MSB, static_cast<uint8_t>(bitRateReg >> 8));
    writeRegister(REG_BITRATE_LSB, static_cast<uint8_t>(bitRateReg));

    // PA_BOOST pin, max output power (+17dBm). No modulation shaping: shaping low-pass filters
    // the OOK edges, which would blur the exact pulse widths this protocol depends on.
    writeRegister(REG_PA_CONFIG, 0x80 | 0x0F);
    writeRegister(REG_PA_RAMP, 0x08); // ModulationShaping=00 (none), PaRamp=50us

    // No preamble/sync word/CRC: the FIFO payload we stream in transmitWaveform() *is* the exact
    // on-air waveform, byte for byte.
    writeRegister(REG_PREAMBLE_MSB, 0x00);
    writeRegister(REG_PREAMBLE_LSB, 0x00);
    writeRegister(REG_SYNC_CONFIG, 0x00);    // SyncOn=0
    writeRegister(REG_PACKET_CONFIG1, 0x00); // fixed length, DcFree=none, CrcOn=0
    writeRegister(REG_FIFO_THRESH, FIFO_THRESHOLD); // TxStartCondition=0 (wait for FifoLevel)
}

void SX1276Driver::init()
{
    resetChip();
    configureDioPins();

    spi_bus_config_t busConfig = {};
    busConfig.mosi_io_num = GPIO_NUM_23;
    busConfig.miso_io_num = GPIO_NUM_19;
    busConfig.sclk_io_num = GPIO_NUM_18;
    busConfig.quadwp_io_num = -1;
    busConfig.quadhd_io_num = -1;
    esp_err_t err = spi_bus_initialize(VSPI_HOST, &busConfig, SPI_DMA_DISABLED);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialise VSPI bus: %d", err);
        return;
    }

    spi_device_interface_config_t devConfig = {};
    devConfig.clock_speed_hz = SPI_CLOCK_HZ;
    devConfig.mode = 0; // CPOL=0, CPHA=0
    devConfig.spics_io_num = SX1276_NSS_GPIO;
    devConfig.queue_size = 1;
    err = spi_bus_add_device(VSPI_HOST, &devConfig, &spiDevice);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add SX1276 as a SPI device: %d", err);
        return;
    }

    uint8_t version = readRegister(REG_VERSION);
    ESP_LOGI(TAG, "SX1276 RegVersion=0x%02X", version);
    if (version == 0x00 || version == 0xFF)
    {
        ESP_LOGE(TAG, "SX1276 not responding on SPI, aborting init");
        spiDevice = nullptr;
        return;
    }

    configureRadio();
    ESP_LOGI(TAG, "SX1276 initialised");
}

void SX1276Driver::transmitWaveform(const std::vector<uint8_t>& waveform)
{
    size_t total = waveform.size();

    writeRegister(REG_PACKET_CONFIG2, 0x40 | static_cast<uint8_t>((total >> 8) & 0x07));
    writeRegister(REG_PAYLOAD_LENGTH, static_cast<uint8_t>(total & 0xFF));

    writeRegister(REG_OP_MODE, OPMODE_STANDBY);

    // Pre-fill the FIFO before enabling Tx, per the datasheet's "Handling Large Packets" sequence.
    size_t sent = 0;
    size_t preload = std::min(total, FIFO_CAPACITY);
    while (sent < preload)
    {
        size_t chunk = std::min(preload - sent, FIFO_CHUNK);
        writeFifo(waveform.data() + sent, chunk);
        sent += chunk;
    }

    writeRegister(REG_OP_MODE, OPMODE_TX);

    int64_t deadline = esp_timer_get_time() + FIFO_WAIT_TIMEOUT_US;
    while (sent < total)
    {
        if (gpio_get_level(SX1276_DIO1_GPIO) == 0) // FifoLevel cleared: level has drained to the threshold
        {
            size_t chunk = std::min(total - sent, FIFO_CHUNK);
            writeFifo(waveform.data() + sent, chunk);
            sent += chunk;
            deadline = esp_timer_get_time() + FIFO_WAIT_TIMEOUT_US;
        }
        else if (esp_timer_get_time() > deadline)
        {
            ESP_LOGE(TAG, "Timed out refilling SX1276 FIFO, aborting transmission");
            writeRegister(REG_OP_MODE, OPMODE_STANDBY);
            return;
        }
    }

    deadline = esp_timer_get_time() + FIFO_WAIT_TIMEOUT_US;
    while (gpio_get_level(SX1276_DIO0_GPIO) == 0) // PacketSent
    {
        if (esp_timer_get_time() > deadline)
        {
            ESP_LOGE(TAG, "Timed out waiting for SX1276 PacketSent");
            break;
        }
    }

    writeRegister(REG_OP_MODE, OPMODE_STANDBY);
}

void SX1276Driver::send(const std::array<uint8_t, 5>& data, uint8_t repeats)
{
    ESP_LOGI(TAG, "Sending %02X %02X %02X %02X %02X, repeats=%d",
             data[0], data[1], data[2], data[3], data[4], repeats);

    if (spiDevice == nullptr)
    {
        ESP_LOGE(TAG, "SX1276 not initialised, dropping command");
        return;
    }

    std::vector<uint8_t> waveform = buildWaveform(data);

    for (uint8_t i = 0; i < repeats; ++i)
    {
        transmitWaveform(waveform);
        if (i + 1 < repeats)
        {
            esp_rom_delay_us(RESET_US);
        }
    }
}
