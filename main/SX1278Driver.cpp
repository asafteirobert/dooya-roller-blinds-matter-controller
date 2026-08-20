#include "SX1278Driver.hpp"
#include "Constants.hpp"

#include <algorithm>
#include <utility>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_attr.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace
{
// logRxEvent() runs from ISR context and may run with the flash cache disabled (e.g. mid-NVS-read,
// see the crash this fixed: "Cache disabled but cached memory region accessed" from ets_printf
// dereferencing SX1278Driver::TAG). ESP_DRAM_LOG* only keeps its *format* string out of flash --
// the tag argument is printed as-is, so it also needs to live in real DRAM rather than the normal
// flash-mapped rodata a plain string literal/constexpr char* would land in.
static const DRAM_ATTR char ISR_LOG_TAG[] = "SX1278Driver";

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

// The SX1278's FSK/OOK FIFO is only 64 bytes deep, so any payload longer than that (ours always
// is) must be streamed in while transmitting, per the "Handling Large Packets" procedure in the
// datasheet: pre-fill, enable Tx, then keep topping up in FIFO_CHUNK-sized bursts whenever the
// FifoLevel flag (mirrored on DIO1) says the level has dropped back to the threshold.
constexpr size_t FIFO_CAPACITY = 64;
constexpr size_t FIFO_CHUNK = 32;
constexpr uint8_t FIFO_THRESHOLD = FIFO_CAPACITY - FIFO_CHUNK; // leaves exactly one chunk of headroom

constexpr int64_t FIFO_WAIT_TIMEOUT_US = 500000; // generous vs. the ~60-90 ms a real transfer takes

// Deep enough to absorb a burst of UI/Matter-driven commands without send() ever blocking on air time.
constexpr UBaseType_t COMMAND_QUEUE_LENGTH = 16;
constexpr uint32_t SENDER_TASK_STACK_WORDS = 4096;
constexpr UBaseType_t SENDER_TASK_PRIORITY = 5;

// Minimum gap between the end of one queued command and the start of the next
constexpr uint32_t COMMAND_GAP_MS = 200;

// Deep enough to absorb a burst of repeated button presses (real remotes send several repeats
// per press) without the ISR ever blocking or dropping a decoded frame.
constexpr UBaseType_t RX_FRAME_QUEUE_LENGTH = 8;
constexpr uint32_t RECEIVER_TASK_STACK_WORDS = 4096;
constexpr UBaseType_t RECEIVER_TASK_PRIORITY = 5;

// --- OOK/PWM RX decode tolerances, mirroring buildWaveform()'s encoding in reverse. Only the
// duration of each HIGH ("mark") run is used to decode a bit -- LONG_US means 1, SHORT_US means
// 0 -- since buildWaveform() always pairs a mark with the complementary-length space to keep the
// bit period constant, so the space carries no extra information.
//
// Every classification band below is separated from its neighbours by a dead zone that belongs to
// neither: a duration landing in a gap aborts the in-progress frame (see handleReceivedEdge)
// instead of being forced into whichever band happens to be closest. Without that gap, jitter that
// pushes a single mark across a boundary silently flips that bit instead of dropping the frame --
// this is what let corrupted-but-well-formed frames (right length, wrong bit) through and produced
// commands with a plausible-looking but wrong remote ID. A dropped frame is harmless: the remote
// repeats every button press several times and RadioController only needs one good frame to get
// through.
constexpr int64_t RX_SHORT_MIN_US = 180;
constexpr int64_t RX_SHORT_MAX_US = 460; // SHORT_US(350) + ~31%
constexpr int64_t RX_LONG_MIN_US = 590;  // LONG_US(700) - ~16% -- 130us dead zone vs RX_SHORT_MAX_US
constexpr int64_t RX_LONG_MAX_US = 1000; // LONG_US(700) + ~43%, comfortably below RX_SYNC_MIN_US
constexpr int64_t RX_SYNC_MIN_US = 2800; // SYNC_US(4600) - ~39%, comfortably above RX_LONG_MAX_US
constexpr int64_t RX_SYNC_MAX_US = 6200; // SYNC_US(4600) + ~35%
constexpr uint8_t RX_FRAME_BITS = 40;

// Below RX_SHORT_MIN_US by a wide margin, so a run this brief can never be a real mark or space --
// it's an RF/electrical glitch on DIO2. An edge is only applied to rxState once it has survived
// this long without a second edge closing it (see handleReceivedEdge()/rxPendingValid), so a spike
// that dips high->low->high or low->high->low within this window never reaches the state machine
// on either edge; the run it interrupted is read as one continuous run instead of two.
constexpr int64_t RX_NOISE_MAX_US = 50;

// The sync mark is always followed by a fixed 2*LONG_US (1400us) low gap before the first bit's
// mark starts (see buildWaveform()). Requiring that gap to also match before committing to
// "in frame" makes a random noise glitch that happens to land in the SYNC_US window (rare on its
// own) need a *second* coincidence to be mistaken for a real frame, which is what lets a weak
// real signal be told apart from noise reliably instead of just by luck.
constexpr int64_t RX_POST_SYNC_GAP_MIN_US = 1050;
constexpr int64_t RX_POST_SYNC_GAP_MAX_US = 1850;

// --- SX1278 FSK/OOK register map (see RFM9x/SX1278 datasheet section 6.2) ---
constexpr uint8_t REG_FIFO = 0x00;
constexpr uint8_t REG_OP_MODE = 0x01;
constexpr uint8_t REG_BITRATE_MSB = 0x02;
constexpr uint8_t REG_BITRATE_LSB = 0x03;
constexpr uint8_t REG_FRF_MSB = 0x06;
constexpr uint8_t REG_FRF_MID = 0x07;
constexpr uint8_t REG_FRF_LSB = 0x08;
constexpr uint8_t REG_PA_CONFIG = 0x09;
constexpr uint8_t REG_PA_RAMP = 0x0A;
constexpr uint8_t REG_RX_CONFIG = 0x0D;
constexpr uint8_t REG_RX_BW = 0x12;
constexpr uint8_t REG_OOK_PEAK = 0x14;
constexpr uint8_t REG_OOK_FIX = 0x15;
constexpr uint8_t REG_RSSI_VALUE = 0x11;
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
constexpr uint8_t OPMODE_RX_CONTINUOUS = MODULATION_OOK_LF | 0x05;

// RegPacketConfig2: DataMode=1 (Packet, used for Tx) vs. DataMode=0 (Continuous, used for Rx --
// our protocol has no SX1278-recognised preamble/sync word for the packet engine to frame it
// with, so DIO2 is read as a raw envelope and decoded in software instead; see
// handleReceivedEdge).
constexpr uint8_t PACKET_CONFIG2_DATA_MODE_PACKET = 0x40;
constexpr uint8_t PACKET_CONFIG2_DATA_MODE_CONTINUOUS = 0x00;

// Appends single-value-per-chip runs to a byte buffer, MSB-first, matching the bit order the
// SX1278 shifts a FIFO byte out in 
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

void SX1278Driver::resetChip()
{
    gpio_config_t resetConfig = {};
    resetConfig.pin_bit_mask = 1ULL << SX1278_RESET_GPIO;
    resetConfig.mode = GPIO_MODE_OUTPUT;
    resetConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    resetConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    resetConfig.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&resetConfig);

    // Manual reset per datasheet section 7.2.2: NRESET low for >100us, then wait >=5ms.
    gpio_set_level(SX1278_RESET_GPIO, 0);
    esp_rom_delay_us(200);
    gpio_set_level(SX1278_RESET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void SX1278Driver::configureDioPins()
{
    // DIO0 = PacketSent, DIO1 = FifoLevel in Tx/packet mode -- both are the default DioMapping
    // (RegDioMapping1 reset value 0x00), so only the GPIO direction needs configuring here.
    gpio_config_t dioConfig = {};
    dioConfig.pin_bit_mask = (1ULL << SX1278_DIO0_GPIO) | (1ULL << SX1278_DIO1_GPIO);
    dioConfig.mode = GPIO_MODE_INPUT;
    dioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    dioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    dioConfig.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&dioConfig);

    // DIO2 = Data: with DataMode=Continuous (entered only while receiving, see
    // enterReceiveMode()) this pin mirrors the raw OOK-demodulated envelope in real time, every
    // edge of which is timestamped and decoded by handleReceivedEdge(). Its interrupt starts
    // disabled since DIO2's meaning while transmitting/mid-configuration is undefined;
    // enterReceiveMode() is what turns it on.
    gpio_config_t dio2Config = {};
    dio2Config.pin_bit_mask = 1ULL << SX1278_DIO2_GPIO;
    dio2Config.mode = GPIO_MODE_INPUT;
    dio2Config.pull_up_en = GPIO_PULLUP_DISABLE;
    dio2Config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    dio2Config.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&dio2Config);

    // ButtonDriver's iot_button component may already have installed this shared service (it
    // runs first, see app_main.cpp); ESP_ERR_INVALID_STATE just means it's already up.
    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %d", err);
        return;
    }
    err = gpio_isr_handler_add(SX1278_DIO2_GPIO, &SX1278Driver::dio2IsrHandler, this);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add DIO2 ISR handler: %d", err);
        return;
    }
    gpio_intr_disable(SX1278_DIO2_GPIO);
}

void SX1278Driver::writeRegister(uint8_t address, uint8_t value)
{
    uint8_t buffer[2] = { static_cast<uint8_t>(address | SPI_WRITE_BIT), value };
    spi_transaction_t transaction = {};
    transaction.length = 8 * sizeof(buffer);
    transaction.tx_buffer = buffer;
    spi_device_polling_transmit(this->spiDevice, &transaction);
}

uint8_t SX1278Driver::readRegister(uint8_t address)
{
    uint8_t tx[2] = { static_cast<uint8_t>(address & ~SPI_WRITE_BIT), 0x00 };
    uint8_t rx[2] = {};
    spi_transaction_t transaction = {};
    transaction.length = 8 * sizeof(tx);
    transaction.tx_buffer = tx;
    transaction.rx_buffer = rx;
    spi_device_polling_transmit(this->spiDevice, &transaction);
    return rx[1];
}

void SX1278Driver::writeFifo(const uint8_t* data, size_t length)
{
    uint8_t buffer[FIFO_CHUNK + 1];
    buffer[0] = REG_FIFO | SPI_WRITE_BIT;
    std::copy(data, data + length, buffer + 1);

    spi_transaction_t transaction = {};
    transaction.length = 8 * (length + 1);
    transaction.tx_buffer = buffer;
    spi_device_polling_transmit(this->spiDevice, &transaction);
}

void SX1278Driver::configureRadio()
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

// One-time OOK receiver setup. These registers are independent of the Tx/Rx DataMode toggle in
// enterReceiveMode()/transmitWaveform(), so they only need writing once.
void SX1278Driver::configureReceiver()
{
    // BitSyncOn=0, OokThreshType=01 (Peak, recommended default). BitSyncOn must be off: our
    // protocol has no 0x55/0xAA preamble for the bit synchronizer to lock onto, so leaving it on
    // would gate DIO2/Data to a recovered bit clock we can't produce, instead of the raw envelope
    // handleReceivedEdge() needs (datasheet section 4.2.3.3/2.1.12.3).
    writeRegister(REG_OOK_PEAK, 0x08);

    // ~10.4kHz single-side channel filter bandwidth (RxBwMant=24, RxBwExp=5) -- this is the same
    // bandwidth the datasheet itself quotes its OOK sensitivity figures at (Table 8), and
    // meaningfully cuts the noise power reaching the demodulator compared to a wider filter,
    // while still settling far faster than our shortest pulse (SHORT_US=350us). If reception
    // stops working *entirely* after this change rather than just staying weak, that points at
    // the remote's actual carrier sitting outside this narrower passband (crystal tolerance)
    // rather than at noise -- widen it back up (e.g. 0x0B for ~50kHz) to check.
    writeRegister(REG_RX_BW, 0x15);

    // Floor threshold for the OOK Peak demodulator: how far above the noise floor DIO2 must rise
    // before it's treated as a real signal rather than noise. The right value is inherently
    // specific to this board/antenna/environment -- the datasheet's own recommended procedure
    // (section 2.1.3.2, "Optimizing the Floor Threshold") is to raise it until DIO2 stops
    // toggling with no transmitter active. This is only a conservative starting point (double
    // the POR reset value); use the "sx1278reg" console command to tune it properly on this
    // hardware without reflashing, e.g. `sx1278reg 0x15 0x20`.
    writeRegister(REG_OOK_FIX, 0x43);

    // AgcAutoOn=1, RxTrigger=001 (Rssi interrupt): the LNA gain (re-)converges whenever RSSI
    // crosses the threshold, i.e. whenever a transmission begins after a silent gap -- exactly
    // the pattern a burst of remote button presses produces during otherwise-idle continuous Rx.
    writeRegister(REG_RX_CONFIG, 0x09);
}

void SX1278Driver::init()
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
    devConfig.spics_io_num = SX1278_NSS_GPIO;
    devConfig.queue_size = 1;
    err = spi_bus_add_device(VSPI_HOST, &devConfig, &this->spiDevice);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add SX1278 as a SPI device: %d", err);
        return;
    }

    uint8_t version = readRegister(REG_VERSION);
    ESP_LOGI(TAG, "SX1278 RegVersion=0x%02X", version);
    if (version == 0x00 || version == 0xFF)
    {
        ESP_LOGE(TAG, "SX1278 not responding on SPI, aborting init");
        this->spiDevice = nullptr;
        return;
    }

    configureRadio();
    configureReceiver();
    ESP_LOGI(TAG, "SX1278 initialised");

    this->commandQueue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(QueuedCommand));
    this->rxFrameQueue = xQueueCreate(RX_FRAME_QUEUE_LENGTH, sizeof(std::array<uint8_t, 5>));
    xTaskCreate(&SX1278Driver::receiverTask, "sx1278_receiver", RECEIVER_TASK_STACK_WORDS, this,
                RECEIVER_TASK_PRIORITY, &this->receiverTaskHandle);
    xTaskCreate(&SX1278Driver::senderTask, "sx1278_sender", SENDER_TASK_STACK_WORDS, this, SENDER_TASK_PRIORITY,
                &this->senderTaskHandle);

    // Flushes a still-pending RX edge once it's survived RX_NOISE_MAX_US without a glitch-close --
    // see the rxPendingValid comment in the header. Needed so the last edge of a burst (no further
    // edge ever arrives to confirm it directly) still gets applied.
    esp_timer_create_args_t noiseTimerArgs = {};
    noiseTimerArgs.callback = &SX1278Driver::noiseTimeoutCallback;
    noiseTimerArgs.arg = this;
    noiseTimerArgs.name = "sx1278_rx_noise";
    esp_timer_create(&noiseTimerArgs, &this->rxNoiseTimer);
}

// Switches the radio into continuous OOK receive mode and (re-)arms the DIO2 interrupt, ready to
// decode the next frame from scratch. Called once at startup and again after every transmission.
void SX1278Driver::enterReceiveMode()
{
    writeRegister(REG_PACKET_CONFIG2, PACKET_CONFIG2_DATA_MODE_CONTINUOUS);
    writeRegister(REG_OP_MODE, OPMODE_STANDBY);
    writeRegister(REG_OP_MODE, OPMODE_RX_CONTINUOUS);

    esp_timer_stop(this->rxNoiseTimer); // no-op (ignored error) if nothing was pending

    portENTER_CRITICAL(&this->rxStateMux);
    this->rxState = RxState{};
    this->rxPendingValid = false;
    portEXIT_CRITICAL(&this->rxStateMux);

    gpio_intr_enable(SX1278_DIO2_GPIO);
}

void SX1278Driver::setReceiveCallback(ReceiveCallback callback)
{
    this->receiveCallback = std::move(callback);
}

bool SX1278Driver::peekRegister(uint8_t address, uint8_t& outValue)
{
    if (this->spiDevice == nullptr)
    {
        return false;
    }
    outValue = readRegister(address);
    return true;
}

bool SX1278Driver::pokeRegister(uint8_t address, uint8_t value)
{
    if (this->spiDevice == nullptr)
    {
        return false;
    }
    writeRegister(address, value);
    return true;
}

bool SX1278Driver::peekRssiDbm(double& outDbm)
{
    uint8_t rssiValue = 0;
    if (!peekRegister(REG_RSSI_VALUE, rssiValue))
    {
        return false;
    }
    outDbm = -static_cast<double>(rssiValue) / 2.0; // RegRssiValue -> dBm, see the datasheet's RSSI section
    return true;
}

// Runs on a dedicated task for the driver's lifetime, invoking the user callback for every frame
// handleReceivedEdge() finishes decoding. Kept off the ISR entirely: the callback ultimately
// reaches BlindController (mutexes) and esp_matter (attribute updates), neither of which is
// ISR-safe.
void SX1278Driver::receiverTask(void* arg)
{
    auto* self = static_cast<SX1278Driver*>(arg);
    std::array<uint8_t, 5> frame;
    while (true)
    {
        if (xQueueReceive(self->rxFrameQueue, &frame, portMAX_DELAY) == pdTRUE && self->receiveCallback)
        {
            self->receiveCallback(frame);
        }
    }
}

void IRAM_ATTR SX1278Driver::dio2IsrHandler(void* arg)
{
    auto* self = static_cast<SX1278Driver*>(arg);
    int64_t now = esp_timer_get_time();
    int level = gpio_get_level(SX1278_DIO2_GPIO);
    self->handleReceivedEdge(now, level);
}

// Applies one confirmed (not a noise glitch) edge to rxState and returns true, with
// outCompletedFrame filled in, if it completes a 40-bit frame; outEvent describes anything
// diagnostically interesting that happened (sync seen, frame dropped and where) for the caller to
// log once it's released rxStateMux -- see the RxEvent comment in the header for why this doesn't
// just log directly. Caller must hold rxStateMux; runs both from ISR context (handleReceivedEdge)
// and task context (noiseTimeoutCallback), so it touches nothing but its arguments and rxState.
//
// Reconstructs the same 40-bit Dooya frame buildWaveform() encodes: a HIGH ("mark") run in the
// SYNC_US range moves to AwaitingSyncGap, the LOW ("space") run that follows it is checked against
// RX_POST_SYNC_GAP_*_US before frame collection actually starts (this second, independent
// coincidence is what tells a real sync pulse apart from a noise glitch that happens to land in
// the SYNC_US window), and every mark after that decodes one payload bit by its duration alone
// (see the RX_*_US tolerance comment above). Anything that doesn't fit an expected width abandons
// the in-progress frame and waits for the next sync pulse, so a corrupted/partial reception
// self-heals on the next button-press repeat instead of needing an explicit timeout.
bool IRAM_ATTR SX1278Driver::applyEdgeToState(int64_t edgeUs, int level, std::array<uint8_t, 5>& outCompletedFrame,
                                               RxEvent& outEvent)
{
    bool frameComplete = false;
    int64_t durationUs = edgeUs - this->rxState.lastEdgeUs;
    this->rxState.lastEdgeUs = edgeUs;
    outEvent = RxEvent{};

    if (level == 0) // falling edge: the HIGH ("mark") run that just ended was durationUs long
    {
        if (durationUs >= RX_SYNC_MIN_US && durationUs <= RX_SYNC_MAX_US)
        {
            if (this->rxState.phase == RxState::Phase::InFrame)
            {
                // A sync-length mark showed up mid-frame instead of a payload bit -- whatever was
                // collected so far is abandoned in favour of chasing this new sync.
                outEvent = { RxEvent::Kind::SyncAbortedFrame, durationUs, this->rxState.bitCount };
            }
            else
            {
                outEvent = { RxEvent::Kind::SyncDetected, durationUs, 0 };
            }
            this->rxState.phase = RxState::Phase::AwaitingSyncGap;
        }
        else if (this->rxState.phase == RxState::Phase::InFrame)
        {
            bool haveBit = true;
            bool bit = false;
            if (durationUs >= RX_SHORT_MIN_US && durationUs <= RX_SHORT_MAX_US)
            {
                bit = false;
            }
            else if (durationUs >= RX_LONG_MIN_US && durationUs <= RX_LONG_MAX_US)
            {
                bit = true;
            }
            else
            {
                // Falls in a dead zone (or wildly out of range): too ambiguous to guess at, so
                // abandon this frame rather than risk committing a wrong bit. Self-heals on the
                // next sync pulse -- either a repeat of the same button press or the next one.
                haveBit = false;
                outEvent = { RxEvent::Kind::BitInvalid, durationUs, this->rxState.bitCount };
                this->rxState.phase = RxState::Phase::Idle;
            }

            if (haveBit)
            {
                size_t byteIndex = this->rxState.bitCount / 8;
                uint8_t bitIndex = 7 - static_cast<uint8_t>(this->rxState.bitCount % 8);
                if (bit)
                {
                    this->rxState.frame[byteIndex] |= static_cast<uint8_t>(1u << bitIndex);
                }
                ++this->rxState.bitCount;

                if (this->rxState.bitCount == RX_FRAME_BITS)
                {
                    this->rxState.phase = RxState::Phase::Idle;
                    frameComplete = true;
                    outCompletedFrame = this->rxState.frame;
                }
            }
        }
        else
        {
            this->rxState.phase = RxState::Phase::Idle;
        }
    }
    else if (this->rxState.phase == RxState::Phase::AwaitingSyncGap)
    {
        // Rising edge: the LOW ("space") run that just ended -- the gap right after the sync
        // mark -- was durationUs long.
        if (durationUs >= RX_POST_SYNC_GAP_MIN_US && durationUs <= RX_POST_SYNC_GAP_MAX_US)
        {
            this->rxState.phase = RxState::Phase::InFrame;
            this->rxState.bitCount = 0;
            this->rxState.frame = {};
        }
        else
        {
            outEvent = { RxEvent::Kind::GapInvalid, durationUs, 0 };
            this->rxState.phase = RxState::Phase::Idle;
        }
    }

    return frameComplete;
}

// Logs an RxEvent produced by applyEdgeToState(). Must only be called *outside* rxStateMux (see
// the RxEvent comment in the header). Safe to call from ISR context with the flash cache disabled:
// ESP_DRAM_LOG* keeps its format string out of flash and (with CONFIG_LOG_IN_IRAM, set for this
// project) resolves to a direct esp_rom_printf call with no heap, no vfs, and no lock -- the same
// pattern ESP-IDF's own low-level drivers (e.g. esp_driver_rmt) use for ISR-adjacent logging. The
// tag passed in must be ISR_LOG_TAG, not the class's TAG -- see its comment.
void IRAM_ATTR SX1278Driver::logRxEvent(const RxEvent& event)
{
    switch (event.kind)
    {
    case RxEvent::Kind::SyncDetected:
        ESP_DRAM_LOGI(ISR_LOG_TAG, "RX: sync pulse detected (%lld us)", static_cast<long long>(event.durationUs));
        break;
    case RxEvent::Kind::SyncAbortedFrame:
        ESP_DRAM_LOGW(ISR_LOG_TAG, "RX: sync pulse seen mid-frame, dropped frame at bit %u (re-sync mark %lld us)",
                      static_cast<unsigned>(event.bitCount), static_cast<long long>(event.durationUs));
        break;
    case RxEvent::Kind::GapInvalid:
        ESP_DRAM_LOGW(ISR_LOG_TAG, "RX: dropped frame before bit 0, post-sync gap %lld us out of range [%lld, %lld]",
                      static_cast<long long>(event.durationUs), static_cast<long long>(RX_POST_SYNC_GAP_MIN_US),
                      static_cast<long long>(RX_POST_SYNC_GAP_MAX_US));
        break;
    case RxEvent::Kind::BitInvalid:
        ESP_DRAM_LOGW(ISR_LOG_TAG, "RX: dropped frame at bit %u, mark duration %lld us out of range",
                      static_cast<unsigned>(event.bitCount), static_cast<long long>(event.durationUs));
        break;
    case RxEvent::Kind::GlitchDropped:
        ESP_DRAM_LOGW(ISR_LOG_TAG, "RX: discarded noise glitch mid-frame at bit %u, edges %lld us apart (< %lld us threshold)",
                      static_cast<unsigned>(event.bitCount), static_cast<long long>(event.durationUs),
                      static_cast<long long>(RX_NOISE_MAX_US));
        break;
    case RxEvent::Kind::None:
        break;
    }
}

// Runs in ISR context on every DIO2 edge while the radio is idling in receive mode (disabled
// during transmission, see transmitWaveform()). Never applies an edge to rxState directly --
// a noise glitch looks identical to a real edge until the *next* edge shows how long the run it
// opened actually lasted, so this only ever buffers the latest edge in rxPendingValid/
// rxPendingEdgeUs/rxPendingLevel and lets applyEdgeToState() see it once it's confirmed real:
//   - if the next edge arrives under RX_NOISE_MAX_US later, the buffered edge and this new one are
//     both noise -- neither is ever applied, and the buffer is simply cleared;
//   - if the next edge arrives RX_NOISE_MAX_US or later, the buffered edge survived long enough to
//     be real -- it's applied now, and this new edge takes its place in the buffer;
//   - if no next edge arrives at all (the last edge of a burst), rxNoiseTimer's timeout applies it
//     from noiseTimeoutCallback instead, once RX_NOISE_MAX_US has passed with nothing to glitch it.
void IRAM_ATTR SX1278Driver::handleReceivedEdge(int64_t nowUs, int level)
{
    bool frameComplete = false;
    std::array<uint8_t, 5> completedFrame{};
    RxEvent rxEvent{};

    portENTER_CRITICAL_ISR(&this->rxStateMux);

    if (this->rxPendingValid && (nowUs - this->rxPendingEdgeUs) < RX_NOISE_MAX_US)
    {
        // Glitch: the buffered edge and this one are both noise. esp_timer_stop() here races the
        // (vanishingly unlikely) case where rxNoiseTimer's timeout is already firing for the
        // buffered edge; if it wins that race regardless, the worst case is one stray bit fed into
        // rxState, which self-heals like any other corrupted frame (see applyEdgeToState).
        esp_timer_stop(this->rxNoiseTimer);
        this->rxPendingValid = false;
        // Only worth logging while a frame is actually being collected -- ambient RF noise glitches
        // constantly while idling between transmissions, and logging every one of those floods the
        // log without telling us anything about dropped packets.
        if (this->rxState.phase == RxState::Phase::InFrame)
        {
            rxEvent = { RxEvent::Kind::GlitchDropped, nowUs - this->rxPendingEdgeUs, this->rxState.bitCount };
        }
    }
    else
    {
        if (this->rxPendingValid)
        {
            esp_timer_stop(this->rxNoiseTimer);
            frameComplete = this->applyEdgeToState(this->rxPendingEdgeUs, this->rxPendingLevel, completedFrame, rxEvent);
        }

        this->rxPendingValid = true;
        this->rxPendingEdgeUs = nowUs;
        this->rxPendingLevel = level;
        esp_timer_start_once(this->rxNoiseTimer, RX_NOISE_MAX_US);
    }

    portEXIT_CRITICAL_ISR(&this->rxStateMux);

    // Logged only after releasing rxStateMux -- see the RxEvent comment in the header.
    if (rxEvent.kind != RxEvent::Kind::None)
    {
        logRxEvent(rxEvent);
    }

    if (frameComplete)
    {
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(this->rxFrameQueue, &completedFrame, &higherPriorityTaskWoken);
        if (higherPriorityTaskWoken == pdTRUE)
        {
            portYIELD_FROM_ISR();
        }
    }
}

// Runs on the esp_timer task, RX_NOISE_MAX_US after the last edge handleReceivedEdge() buffered,
// unless a closer or a later edge got there first and already cleared rxPendingValid (see there).
// Applies that edge the same way a confirming next edge would have -- this is what lets the very
// last edge of a burst (no further DIO2 activity to confirm it) still make it into rxState instead
// of being silently lost while waiting for an edge that's never coming.
void SX1278Driver::noiseTimeoutCallback(void* arg)
{
    auto* self = static_cast<SX1278Driver*>(arg);
    bool frameComplete = false;
    std::array<uint8_t, 5> completedFrame{};
    RxEvent rxEvent{};

    portENTER_CRITICAL(&self->rxStateMux);
    if (self->rxPendingValid)
    {
        frameComplete = self->applyEdgeToState(self->rxPendingEdgeUs, self->rxPendingLevel, completedFrame, rxEvent);
        self->rxPendingValid = false;
    }
    portEXIT_CRITICAL(&self->rxStateMux);

    // Logged only after releasing rxStateMux -- see the RxEvent comment in the header.
    if (rxEvent.kind != RxEvent::Kind::None)
    {
        logRxEvent(rxEvent);
    }

    if (frameComplete)
    {
        xQueueSend(self->rxFrameQueue, &completedFrame, 0);
    }
}

// Runs on a dedicated task for the driver's lifetime, transmitting queued commands strictly in
// submission order so callers never wait on air time. Whenever it isn't actively transmitting,
// it leaves the radio in continuous receive mode listening for the physical remote (see
// enterReceiveMode()).
void SX1278Driver::senderTask(void* arg)
{
    auto* self = static_cast<SX1278Driver*>(arg);
    self->enterReceiveMode();
    QueuedCommand command;
    while (true)
    {
        if (xQueueReceive(self->commandQueue, &command, portMAX_DELAY) == pdTRUE)
        {
            self->sendNow(command.data, command.repeats);
            self->enterReceiveMode();
            vTaskDelay(pdMS_TO_TICKS(COMMAND_GAP_MS));
        }
    }
}

void SX1278Driver::transmitWaveform(const std::vector<uint8_t>& waveform)
{
    // Stop reacting to DIO2 edges and drop out of continuous Rx mode (the sender task may be
    // calling this while the radio is still listening) before touching any packet-framing
    // registers below. Also cancel any RX edge still waiting out RX_NOISE_MAX_US -- left alone, its
    // timeout would otherwise fire mid-transmission and feed a stale, never-really-received edge
    // into rxState (see rxNoiseTimer).
    gpio_intr_disable(SX1278_DIO2_GPIO);
    esp_timer_stop(this->rxNoiseTimer);
    portENTER_CRITICAL(&this->rxStateMux);
    this->rxPendingValid = false;
    portEXIT_CRITICAL(&this->rxStateMux);

    writeRegister(REG_OP_MODE, OPMODE_STANDBY);

    size_t total = waveform.size();

    writeRegister(REG_PACKET_CONFIG2, PACKET_CONFIG2_DATA_MODE_PACKET | static_cast<uint8_t>((total >> 8) & 0x07));
    writeRegister(REG_PAYLOAD_LENGTH, static_cast<uint8_t>(total & 0xFF));

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
        if (gpio_get_level(SX1278_DIO1_GPIO) == 0) // FifoLevel cleared: level has drained to the threshold
        {
            size_t chunk = std::min(total - sent, FIFO_CHUNK);
            writeFifo(waveform.data() + sent, chunk);
            sent += chunk;
            deadline = esp_timer_get_time() + FIFO_WAIT_TIMEOUT_US;
        }
        else if (esp_timer_get_time() > deadline)
        {
            ESP_LOGE(TAG, "Timed out refilling SX1278 FIFO, aborting transmission");
            writeRegister(REG_OP_MODE, OPMODE_STANDBY);
            return;
        }
    }

    deadline = esp_timer_get_time() + FIFO_WAIT_TIMEOUT_US;
    while (gpio_get_level(SX1278_DIO0_GPIO) == 0) // PacketSent
    {
        if (esp_timer_get_time() > deadline)
        {
            ESP_LOGE(TAG, "Timed out waiting for SX1278 PacketSent");
            break;
        }
    }

    writeRegister(REG_OP_MODE, OPMODE_STANDBY);
}

void SX1278Driver::send(const std::array<uint8_t, 5>& data, uint8_t repeats)
{
    if (this->commandQueue == nullptr)
    {
        ESP_LOGE(TAG, "SX1278 not initialised, dropping command");
        return;
    }

    QueuedCommand command{ data, repeats };
    if (xQueueSend(this->commandQueue, &command, 0) != pdTRUE)
    {
        ESP_LOGE(TAG, "SX1278 command queue full, dropping command %02X %02X %02X %02X %02X",
                 data[0], data[1], data[2], data[3], data[4]);
    }
}

// Runs on senderTask. Actually keys the radio and streams the waveform out over SPI; blocks for
// the duration of the transmission (including inter-repeat gaps).
void SX1278Driver::sendNow(const std::array<uint8_t, 5>& data, uint8_t repeats)
{
    ESP_LOGI(TAG, "Sending %02X %02X %02X %02X %02X, repeats=%d",
             data[0], data[1], data[2], data[3], data[4], repeats);

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
