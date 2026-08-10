#include "SX1276Driver.hpp"
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
// bit period constant, so the space carries no extra information. Windows are wide (roughly
// +/-50%) to tolerate the RC transmitter/receiver's timing jitter without overlapping.
constexpr int64_t RX_SHORT_MIN_US = 150;
constexpr int64_t RX_SHORT_MAX_US = 525; // midpoint between SHORT_US and LONG_US
constexpr int64_t RX_LONG_MAX_US = 1200; // comfortably below RX_SYNC_MIN_US
constexpr int64_t RX_SYNC_MIN_US = 2500; // comfortably above RX_LONG_MAX_US
constexpr int64_t RX_SYNC_MAX_US = 6500; // comfortably above SYNC_US
constexpr uint8_t RX_FRAME_BITS = 40;

// The sync mark is always followed by a fixed 2*LONG_US (1400us) low gap before the first bit's
// mark starts (see buildWaveform()). Requiring that gap to also match before committing to
// "in frame" makes a random noise glitch that happens to land in the SYNC_US window (rare on its
// own) need a *second* coincidence to be mistaken for a real frame, which is what lets a weak
// real signal be told apart from noise reliably instead of just by luck.
constexpr int64_t RX_POST_SYNC_GAP_MIN_US = 900;
constexpr int64_t RX_POST_SYNC_GAP_MAX_US = 2100;

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
// our protocol has no SX1276-recognised preamble/sync word for the packet engine to frame it
// with, so DIO2 is read as a raw envelope and decoded in software instead; see
// handleReceivedEdge).
constexpr uint8_t PACKET_CONFIG2_DATA_MODE_PACKET = 0x40;
constexpr uint8_t PACKET_CONFIG2_DATA_MODE_CONTINUOUS = 0x00;

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

    // DIO2 = Data: with DataMode=Continuous (entered only while receiving, see
    // enterReceiveMode()) this pin mirrors the raw OOK-demodulated envelope in real time, every
    // edge of which is timestamped and decoded by handleReceivedEdge(). Its interrupt starts
    // disabled since DIO2's meaning while transmitting/mid-configuration is undefined;
    // enterReceiveMode() is what turns it on.
    gpio_config_t dio2Config = {};
    dio2Config.pin_bit_mask = 1ULL << SX1276_DIO2_GPIO;
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
    err = gpio_isr_handler_add(SX1276_DIO2_GPIO, &SX1276Driver::dio2IsrHandler, this);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add DIO2 ISR handler: %d", err);
        return;
    }
    gpio_intr_disable(SX1276_DIO2_GPIO);
}

void SX1276Driver::writeRegister(uint8_t address, uint8_t value)
{
    uint8_t buffer[2] = { static_cast<uint8_t>(address | SPI_WRITE_BIT), value };
    spi_transaction_t transaction = {};
    transaction.length = 8 * sizeof(buffer);
    transaction.tx_buffer = buffer;
    spi_device_polling_transmit(this->spiDevice, &transaction);
}

uint8_t SX1276Driver::readRegister(uint8_t address)
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

void SX1276Driver::writeFifo(const uint8_t* data, size_t length)
{
    uint8_t buffer[FIFO_CHUNK + 1];
    buffer[0] = REG_FIFO | SPI_WRITE_BIT;
    std::copy(data, data + length, buffer + 1);

    spi_transaction_t transaction = {};
    transaction.length = 8 * (length + 1);
    transaction.tx_buffer = buffer;
    spi_device_polling_transmit(this->spiDevice, &transaction);
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

// One-time OOK receiver setup. These registers are independent of the Tx/Rx DataMode toggle in
// enterReceiveMode()/transmitWaveform(), so they only need writing once.
void SX1276Driver::configureReceiver()
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
    // the POR reset value); use the "sx1276reg" console command to tune it properly on this
    // hardware without reflashing, e.g. `sx1276reg 0x15 0x20`.
    writeRegister(REG_OOK_FIX, 0x18);

    // AgcAutoOn=1, RxTrigger=001 (Rssi interrupt): the LNA gain (re-)converges whenever RSSI
    // crosses the threshold, i.e. whenever a transmission begins after a silent gap -- exactly
    // the pattern a burst of remote button presses produces during otherwise-idle continuous Rx.
    writeRegister(REG_RX_CONFIG, 0x09);
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
    err = spi_bus_add_device(VSPI_HOST, &devConfig, &this->spiDevice);
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
        this->spiDevice = nullptr;
        return;
    }

    configureRadio();
    configureReceiver();
    ESP_LOGI(TAG, "SX1276 initialised");

    this->commandQueue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(QueuedCommand));
    this->rxFrameQueue = xQueueCreate(RX_FRAME_QUEUE_LENGTH, sizeof(std::array<uint8_t, 5>));
    xTaskCreate(&SX1276Driver::receiverTask, "sx1276_receiver", RECEIVER_TASK_STACK_WORDS, this,
                RECEIVER_TASK_PRIORITY, &this->receiverTaskHandle);
    xTaskCreate(&SX1276Driver::senderTask, "sx1276_sender", SENDER_TASK_STACK_WORDS, this, SENDER_TASK_PRIORITY,
                &this->senderTaskHandle);
}

// Switches the radio into continuous OOK receive mode and (re-)arms the DIO2 interrupt, ready to
// decode the next frame from scratch. Called once at startup and again after every transmission.
void SX1276Driver::enterReceiveMode()
{
    writeRegister(REG_PACKET_CONFIG2, PACKET_CONFIG2_DATA_MODE_CONTINUOUS);
    writeRegister(REG_OP_MODE, OPMODE_STANDBY);
    writeRegister(REG_OP_MODE, OPMODE_RX_CONTINUOUS);

    portENTER_CRITICAL(&this->rxStateMux);
    this->rxState = RxState{};
    portEXIT_CRITICAL(&this->rxStateMux);

    gpio_intr_enable(SX1276_DIO2_GPIO);
}

void SX1276Driver::setReceiveCallback(ReceiveCallback callback)
{
    this->receiveCallback = std::move(callback);
}

bool SX1276Driver::peekRegister(uint8_t address, uint8_t& outValue)
{
    if (this->spiDevice == nullptr)
    {
        return false;
    }
    outValue = readRegister(address);
    return true;
}

bool SX1276Driver::pokeRegister(uint8_t address, uint8_t value)
{
    if (this->spiDevice == nullptr)
    {
        return false;
    }
    writeRegister(address, value);
    return true;
}

bool SX1276Driver::peekRssiDbm(double& outDbm)
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
void SX1276Driver::receiverTask(void* arg)
{
    auto* self = static_cast<SX1276Driver*>(arg);
    std::array<uint8_t, 5> frame;
    while (true)
    {
        if (xQueueReceive(self->rxFrameQueue, &frame, portMAX_DELAY) == pdTRUE && self->receiveCallback)
        {
            self->receiveCallback(frame);
        }
    }
}

void IRAM_ATTR SX1276Driver::dio2IsrHandler(void* arg)
{
    auto* self = static_cast<SX1276Driver*>(arg);
    int64_t now = esp_timer_get_time();
    int level = gpio_get_level(SX1276_DIO2_GPIO);
    self->handleReceivedEdge(now, level);
}

// Runs in ISR context on every DIO2 edge while the radio is idling in receive mode (disabled
// during transmission, see transmitWaveform()). Reconstructs the same 40-bit Dooya frame
// buildWaveform() encodes: a HIGH ("mark") run in the SYNC_US range moves to AwaitingSyncGap, the
// LOW ("space") run that follows it is checked against RX_POST_SYNC_GAP_*_US before frame
// collection actually starts (this second, independent coincidence is what tells a real sync
// pulse apart from a noise glitch that happens to land in the SYNC_US window), and every mark
// after that decodes one payload bit by its duration alone (see the RX_*_US tolerance comment
// above). Anything that doesn't fit an expected width abandons the in-progress frame and waits
// for the next sync pulse, so a corrupted/partial reception self-heals on the next button-press
// repeat instead of needing an explicit timeout.
void IRAM_ATTR SX1276Driver::handleReceivedEdge(int64_t nowUs, int level)
{
    bool frameComplete = false;
    std::array<uint8_t, 5> completedFrame{};

    portENTER_CRITICAL_ISR(&this->rxStateMux);

    int64_t durationUs = nowUs - this->rxState.lastEdgeUs;
    this->rxState.lastEdgeUs = nowUs;

    if (level == 0) // falling edge: the HIGH ("mark") run that just ended was durationUs long
    {
        if (durationUs >= RX_SYNC_MIN_US && durationUs <= RX_SYNC_MAX_US)
        {
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
            else if (durationUs > RX_SHORT_MAX_US && durationUs <= RX_LONG_MAX_US)
            {
                bit = true;
            }
            else
            {
                haveBit = false;
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
                    completedFrame = this->rxState.frame;
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
            this->rxState.phase = RxState::Phase::Idle;
        }
    }

    portEXIT_CRITICAL_ISR(&this->rxStateMux);

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

// Runs on a dedicated task for the driver's lifetime, transmitting queued commands strictly in
// submission order so callers never wait on air time. Whenever it isn't actively transmitting,
// it leaves the radio in continuous receive mode listening for the physical remote (see
// enterReceiveMode()).
void SX1276Driver::senderTask(void* arg)
{
    auto* self = static_cast<SX1276Driver*>(arg);
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

void SX1276Driver::transmitWaveform(const std::vector<uint8_t>& waveform)
{
    // Stop reacting to DIO2 edges and drop out of continuous Rx mode (the sender task may be
    // calling this while the radio is still listening) before touching any packet-framing
    // registers below.
    gpio_intr_disable(SX1276_DIO2_GPIO);
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
    if (this->commandQueue == nullptr)
    {
        ESP_LOGE(TAG, "SX1276 not initialised, dropping command");
        return;
    }

    QueuedCommand command{ data, repeats };
    if (xQueueSend(this->commandQueue, &command, 0) != pdTRUE)
    {
        ESP_LOGE(TAG, "SX1276 command queue full, dropping command %02X %02X %02X %02X %02X",
                 data[0], data[1], data[2], data[3], data[4]);
    }
}

// Runs on senderTask. Actually keys the radio and streams the waveform out over SPI; blocks for
// the duration of the transmission (including inter-repeat gaps).
void SX1276Driver::sendNow(const std::array<uint8_t, 5>& data, uint8_t repeats)
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
