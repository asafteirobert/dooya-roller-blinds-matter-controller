#include "SX1278Driver.hpp"
#include "Constants.hpp"

#include <algorithm>
#include <cinttypes>
#include <utility>

#include <driver/gpio.h>
#include <driver/mcpwm_cap.h>
#include <driver/pulse_cnt.h>
#include <driver/spi_master.h>
#include <esp_attr.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
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

// The FIFO is only 64 bytes deep, so any payload longer than that (ours always is) is streamed in
// while transmitting per the datasheet's "Handling Large Packets" procedure: pre-fill, enable Tx,
// then top up in FIFO_CHUNK bursts whenever FifoLevel (mirrored on DIO1) drops to the threshold.
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

// Deep enough to absorb a burst of repeated button presses without the ISR ever blocking/dropping.
constexpr UBaseType_t RX_FRAME_QUEUE_LENGTH = 8;
constexpr uint32_t RECEIVER_TASK_STACK_WORDS = 4096;
constexpr UBaseType_t RECEIVER_TASK_PRIORITY = 5;

// dio2SetupTask() runs the MCPWM/PCNT setup from core 1 (see there) so their interrupts bind away
// from WiFi/BLE's dispatch, which is pinned to core 0.
constexpr uint32_t DIO2_SETUP_TASK_STACK_WORDS = 3072;
constexpr UBaseType_t DIO2_SETUP_TASK_PRIORITY = 5;
constexpr BaseType_t DIO2_SETUP_CORE = 1;

struct Dio2SetupContext
{
    SX1278Driver* driver;
    SemaphoreHandle_t doneSemaphore;
};

// ESP32's PCNT counter register is a signed 16-bit field.
constexpr int PCNT_LOW_LIMIT = -32768;
constexpr int PCNT_HIGH_LIMIT = 32767;
// handleReceivedEdge() resets rxIsrEdgeCount (and the PCNT counter) back to 0 once it reaches this,
// well below the hardware ceiling above -- see there. Any value comfortably below the ceiling works.
constexpr int PCNT_RESET_THRESHOLD = 20000;

// --- OOK/PWM RX decode tolerances, mirroring buildWaveform()'s encoding in reverse. Only each
// HIGH ("mark") run's duration is decoded -- LONG_US means 1, SHORT_US means 0 -- since the
// complementary space carries no extra information. Each band is separated from its neighbours by
// a dead zone: a duration landing in a gap aborts the in-progress frame (see handleReceivedEdge)
// instead of being forced into the nearest band, so jitter can't silently flip a bit into a
// plausible-but-wrong frame. A dropped frame is harmless -- the remote repeats every press.
constexpr int64_t RX_SHORT_MIN_US = 180;
constexpr int64_t RX_SHORT_MAX_US = 460; // SHORT_US(350) + ~31%
constexpr int64_t RX_LONG_MIN_US = 590;  // LONG_US(700) - ~16% -- 130us dead zone vs RX_SHORT_MAX_US
constexpr int64_t RX_LONG_MAX_US = 1000; // LONG_US(700) + ~43%, comfortably below RX_SYNC_MIN_US
constexpr int64_t RX_SYNC_MIN_US = 2800; // SYNC_US(4600) - ~39%, comfortably above RX_LONG_MAX_US
constexpr int64_t RX_SYNC_MAX_US = 6200; // SYNC_US(4600) + ~35%
constexpr uint8_t RX_FRAME_BITS = 40;

// Below RX_SHORT_MIN_US by a wide margin, so a run this brief is always an RF/electrical glitch,
// never a real mark or space -- see rxPendingValid.
constexpr int64_t RX_NOISE_MAX_US = 50;

// The sync mark is always followed by a fixed 2*LONG_US (1400us) low gap before the first bit's
// mark (see buildWaveform()). Requiring that gap to also match before committing to "in frame"
// needs a noise glitch to land in *two* independent windows to be mistaken for a real frame.
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

// RegPacketConfig2 DataMode: Packet for Tx, Continuous for Rx (our protocol has no SX1278-framable
// preamble/sync word, so DIO2 is read as a raw envelope and decoded in software instead).
constexpr uint8_t PACKET_CONFIG2_DATA_MODE_PACKET = 0x40;
constexpr uint8_t PACKET_CONFIG2_DATA_MODE_CONTINUOUS = 0x00;

// Appends single-value-per-chip runs to a byte buffer, MSB-first, matching the bit order the
// SX1278 shifts a FIFO byte out in.
class BitWriter
{
public:
    explicit BitWriter(std::vector<uint8_t>& buffer) : buffer_(buffer) {}

    void appendRun(bool level, uint32_t chipCount)
    {
        for (uint32_t i = 0; i < chipCount; ++i)
        {
            size_t byteIndex = this->bitCount_ / 8;
            uint8_t bitIndex = 7 - static_cast<uint8_t>(this->bitCount_ % 8);
            if (byteIndex == this->buffer_.size())
            {
                this->buffer_.push_back(0);
            }
            if (level)
            {
                this->buffer_[byteIndex] |= static_cast<uint8_t>(1u << bitIndex);
            }
            ++this->bitCount_;
        }
    }

private:
    std::vector<uint8_t>& buffer_;
    size_t bitCount_ = 0;
};

// Encodes the 40-bit Dooya command into the literal on-air OOK waveform for a single frame
// (sync + 40 bits, each chip SAMPLE_US wide).
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
            // Mark and space swap between LONG/SHORT so every bit takes the same total time.
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

// Sets up the PCNT ground-truth edge counter for DIO2 -- see rxEdgeCountUnit in the header. Must
// run before configureDioPins(): pcnt_new_channel() force-enables DIO2's pull-up, and
// configureDioPins()'s mcpwm_new_capture_channel() disables it again, so that disable must run last.
void SX1278Driver::configureEdgeCounter()
{
    pcnt_unit_config_t unitConfig = {};
    unitConfig.low_limit = PCNT_LOW_LIMIT;
    unitConfig.high_limit = PCNT_HIGH_LIMIT;
    esp_err_t err = pcnt_new_unit(&unitConfig, &this->rxEdgeCountUnit);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create DIO2 edge-count PCNT unit: %d", err);
        return;
    }

    // No glitch filter: this counter must match MCPWM capture's raw per-ISR-call count 1:1,
    // including the noise glitches handleReceivedEdge() later filters out.
    pcnt_chan_config_t chanConfig = {};
    chanConfig.edge_gpio_num = SX1278_DIO2_GPIO;
    chanConfig.level_gpio_num = -1;
    err = pcnt_new_channel(this->rxEdgeCountUnit, &chanConfig, &this->rxEdgeCountChannel);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create DIO2 edge-count PCNT channel: %d", err);
        return;
    }

    // Count both edges, matching flags.pos_edge/neg_edge in configureDioPins()'s capture channel.
    pcnt_channel_set_edge_action(this->rxEdgeCountChannel, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                  PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(this->rxEdgeCountChannel, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                   PCNT_CHANNEL_LEVEL_ACTION_KEEP);

    err = pcnt_unit_enable(this->rxEdgeCountUnit);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable DIO2 edge-count PCNT unit: %d", err);
        return;
    }
    pcnt_unit_clear_count(this->rxEdgeCountUnit);
    // Not started here -- enterReceiveMode()/transmitWaveform() start/stop it in lockstep with the
    // MCPWM capture channel.
}

// Runs configureEdgeCounter()/configureDioPins() from a task pinned to core 1 instead of directly
// from init() (core 0), so the interrupts they register bind to core 1 instead of contending with
// WiFi/BLE on core 0 -- see DIO2_SETUP_* above. init() blocks on doneSemaphore to stay synchronous.
void SX1278Driver::dio2SetupTask(void* arg)
{
    auto* context = static_cast<Dio2SetupContext*>(arg);
    context->driver->configureEdgeCounter();
    context->driver->configureDioPins();
    xSemaphoreGive(context->doneSemaphore);
    vTaskDelete(nullptr);
}

void SX1278Driver::configureDioPins()
{
    // DIO0 = PacketSent, DIO1 = FifoLevel in Tx/packet mode -- both are the default DioMapping, so
    // only the GPIO direction needs configuring here.
    gpio_config_t dioConfig = {};
    dioConfig.pin_bit_mask = (1ULL << SX1278_DIO0_GPIO) | (1ULL << SX1278_DIO1_GPIO);
    dioConfig.mode = GPIO_MODE_INPUT;
    dioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    dioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    dioConfig.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&dioConfig);

    // DIO2 = Data: in DataMode=Continuous (receive only) this pin mirrors the raw OOK-demodulated
    // envelope. Every edge is timestamped by the MCPWM capture peripheral, latched atomically in
    // hardware so the timestamp can't be corrupted by ISR scheduling delay the way a software
    // esp_timer_get_time() read could be under WiFi/BLE interrupt pressure (both pinned to core 0
    // alongside this driver's init) -- that used to produce spurious multi-millisecond gaps that
    // broke frame decoding. Starts disabled since DIO2's meaning while transmitting is undefined;
    // enterReceiveMode() turns it on.
    mcpwm_capture_timer_config_t capTimerConfig = {};
    capTimerConfig.group_id = 0;
    capTimerConfig.clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
    // Best-effort hint only: on plain ESP32 this is ignored (clock is hardwired to APB) -- the
    // actual resolution is read back just below instead.
    capTimerConfig.resolution_hz = 1'000'000;
    esp_err_t err = mcpwm_new_capture_timer(&capTimerConfig, &this->rxCaptureTimer);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create DIO2 capture timer: %d", err);
        return;
    }

    uint32_t captureResolutionHz = 0;
    err = mcpwm_capture_timer_get_resolution(this->rxCaptureTimer, &captureResolutionHz);
    if (err != ESP_OK || captureResolutionHz == 0 || captureResolutionHz % 1'000'000 != 0)
    {
        ESP_LOGE(TAG, "Unusable DIO2 capture timer resolution: %" PRIu32 " Hz (err=%d)", captureResolutionHz, err);
        return;
    }
    this->rxCaptureTicksPerUs = captureResolutionHz / 1'000'000;

    mcpwm_capture_channel_config_t capChanConfig = {};
    capChanConfig.gpio_num = SX1278_DIO2_GPIO;
    capChanConfig.intr_priority = 3; // max level MCPWM capture can ever get -- see dio2SetupTask()
    capChanConfig.prescale = 1;
    capChanConfig.flags.pos_edge = true;
    capChanConfig.flags.neg_edge = true;
    err = mcpwm_new_capture_channel(this->rxCaptureTimer, &capChanConfig, &this->rxCaptureChannel);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create DIO2 capture channel: %d", err);
        return;
    }

    mcpwm_capture_event_callbacks_t capCallbacks = { .on_cap = &SX1278Driver::onDio2Capture };
    err = mcpwm_capture_channel_register_event_callbacks(this->rxCaptureChannel, &capCallbacks, this);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register DIO2 capture callback: %d", err);
        return;
    }

    err = mcpwm_capture_timer_enable(this->rxCaptureTimer);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable DIO2 capture timer: %d", err);
        return;
    }
    err = mcpwm_capture_timer_start(this->rxCaptureTimer);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start DIO2 capture timer: %d", err);
        return;
    }
    // Not enabled here -- a freshly created channel starts disabled; enterReceiveMode() enables it.
}

void SX1278Driver::configureDebugPins()
{
    gpio_config_t debugConfig = {};
    debugConfig.pin_bit_mask = (1ULL << SX1278_RX_DEBUG_1_GPIO) | (1ULL << SX1278_RX_DEBUG_2_GPIO);
    debugConfig.mode = GPIO_MODE_OUTPUT;
    debugConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    debugConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    debugConfig.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&debugConfig);
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
    // LongRangeMode can only change while the *target* mode is Sleep, so force that first.
    this->writeRegister(REG_OP_MODE, 0x00);
    this->writeRegister(REG_OP_MODE, OPMODE_SLEEP);
    this->writeRegister(REG_OP_MODE, OPMODE_STANDBY);
    vTaskDelay(pdMS_TO_TICKS(10)); // crystal oscillator startup

    uint32_t frf = static_cast<uint32_t>((RF_FREQUENCY_HZ * 524288.0) / FXOSC_HZ + 0.5);
    this->writeRegister(REG_FRF_MSB, static_cast<uint8_t>(frf >> 16));
    this->writeRegister(REG_FRF_MID, static_cast<uint8_t>(frf >> 8));
    this->writeRegister(REG_FRF_LSB, static_cast<uint8_t>(frf));

    // Bit rate sets the on-air duration of a single chip in our hand-encoded waveform; it has
    // nothing to do with any "real" data rate.
    uint32_t bitRateReg = static_cast<uint32_t>(FXOSC_HZ / BIT_RATE_BPS + 0.5);
    this->writeRegister(REG_BITRATE_MSB, static_cast<uint8_t>(bitRateReg >> 8));
    this->writeRegister(REG_BITRATE_LSB, static_cast<uint8_t>(bitRateReg));

    // PA_BOOST pin, max output power (+17dBm). No modulation shaping: shaping would low-pass
    // filter the OOK edges, blurring the exact pulse widths this protocol depends on.
    this->writeRegister(REG_PA_CONFIG, 0x80 | 0x0F);
    this->writeRegister(REG_PA_RAMP, 0x08); // ModulationShaping=00 (none), PaRamp=50us

    // No preamble/sync word/CRC: the FIFO payload streamed in transmitWaveform() *is* the exact
    // on-air waveform, byte for byte.
    this->writeRegister(REG_PREAMBLE_MSB, 0x00);
    this->writeRegister(REG_PREAMBLE_LSB, 0x00);
    this->writeRegister(REG_SYNC_CONFIG, 0x00);    // SyncOn=0
    this->writeRegister(REG_PACKET_CONFIG1, 0x00); // fixed length, DcFree=none, CrcOn=0
    this->writeRegister(REG_FIFO_THRESH, FIFO_THRESHOLD); // TxStartCondition=0 (wait for FifoLevel)
}

// One-time OOK receiver setup, independent of the Tx/Rx DataMode toggle in
// enterReceiveMode()/transmitWaveform().
void SX1278Driver::configureReceiver()
{
    // BitSyncOn=0 (no preamble for the bit synchronizer to lock onto -- we need the raw envelope,
    // not a recovered bit clock), OokThreshType=01 (Peak, recommended default).
    this->writeRegister(REG_OOK_PEAK, 0x08);

    // single-side channel filter bandwidth
    // Value	Mant	Exp	Bandwidth
    // 0x17 	24	    7	2.6 kHz
    // 0x16	    24	    6	5.2 kHz (current)
    // 0x15	    24	    5	10.4 kHz
    // 0x0D	    16	    5	15.6 kHz
    // 0x14	    24	    4	20.8 kHz
    // 0x0C	    16	    4	31.25 kHz
    // 0x0B	    20	    3	50 kHz
    // 0x13	    24	    3	41.7 kHz
    // 0x0A	    20	    2	100 kHz
    // 0x12	    24	    2	83.3 kHz
    this->writeRegister(REG_RX_BW, 0x16);

    // Floor threshold for the OOK Peak demodulator, board/environment-specific -- datasheet
    // section 2.1.3.2 recommends raising it until DIO2 stops toggling with no transmitter active.
    // This is only a conservative starting point; tune live via `sx1278reg 0x15 <value>`.
    this->writeRegister(REG_OOK_FIX, 0x4D);

    // AgcAutoOn=1, RxTrigger=001 (Rssi interrupt): LNA gain reconverges whenever RSSI crosses the
    // threshold, i.e. whenever a transmission begins after a silent gap.
    this->writeRegister(REG_RX_CONFIG, 0x09);
}

void SX1278Driver::init()
{
    this->resetChip();

    // See dio2SetupTask(): runs the MCPWM/PCNT setup from core 1. Blocks until it signals
    // completion so init() stays synchronous from app_main.cpp's perspective.
    SemaphoreHandle_t dio2SetupDone = xSemaphoreCreateBinary();
    Dio2SetupContext dio2SetupContext{ this, dio2SetupDone };
    xTaskCreatePinnedToCore(&SX1278Driver::dio2SetupTask, "sx1278_dio2_setup", DIO2_SETUP_TASK_STACK_WORDS,
                             &dio2SetupContext, DIO2_SETUP_TASK_PRIORITY, nullptr, DIO2_SETUP_CORE);
    xSemaphoreTake(dio2SetupDone, portMAX_DELAY);
    vSemaphoreDelete(dio2SetupDone);

    this->configureDebugPins();

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

    uint8_t version = this->readRegister(REG_VERSION);
    ESP_LOGI(TAG, "SX1278 RegVersion=0x%02X", version);
    if (version == 0x00 || version == 0xFF)
    {
        ESP_LOGE(TAG, "SX1278 not responding on SPI, aborting init");
        this->spiDevice = nullptr;
        return;
    }

    this->configureRadio();
    this->configureReceiver();
    ESP_LOGI(TAG, "SX1278 initialised");

    this->commandQueue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(QueuedCommand));
    this->rxFrameQueue = xQueueCreate(RX_FRAME_QUEUE_LENGTH, sizeof(std::array<uint8_t, 5>));
    xTaskCreate(&SX1278Driver::receiverTask, "sx1278_receiver", RECEIVER_TASK_STACK_WORDS, this,
                RECEIVER_TASK_PRIORITY, &this->receiverTaskHandle);
    xTaskCreate(&SX1278Driver::senderTask, "sx1278_sender", SENDER_TASK_STACK_WORDS, this, SENDER_TASK_PRIORITY,
                &this->senderTaskHandle);

    // Flushes a still-pending RX edge once it's survived RX_NOISE_MAX_US without a glitch-close --
    // see rxPendingValid in the header. Needed so the last edge of a burst still gets applied.
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
    this->writeRegister(REG_PACKET_CONFIG2, PACKET_CONFIG2_DATA_MODE_CONTINUOUS);
    this->writeRegister(REG_OP_MODE, OPMODE_STANDBY);
    this->writeRegister(REG_OP_MODE, OPMODE_RX_CONTINUOUS);

    esp_timer_stop(this->rxNoiseTimer); // no-op (ignored error) if nothing was pending

    portENTER_CRITICAL(&this->rxStateMux);
    this->rxState = RxState{};
    this->rxPendingValid = false;
    this->rxLastReceivedLevel = -1;
    if (this->rxEdgeCountUnit != nullptr)
    {
        pcnt_unit_clear_count(this->rxEdgeCountUnit);
    }
    this->rxIsrEdgeCount = 0;
    portEXIT_CRITICAL(&this->rxStateMux);

    esp_err_t err = mcpwm_capture_channel_enable(this->rxCaptureChannel);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to enable DIO2 capture channel: %d", err);
    }

    if (this->rxEdgeCountUnit != nullptr)
    {
        err = pcnt_unit_start(this->rxEdgeCountUnit);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "Failed to start DIO2 edge-count PCNT unit: %d", err);
        }
    }
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
    outValue = this->readRegister(address);
    return true;
}

bool SX1278Driver::pokeRegister(uint8_t address, uint8_t value)
{
    if (this->spiDevice == nullptr)
    {
        return false;
    }
    this->writeRegister(address, value);
    return true;
}

bool SX1278Driver::peekRssiDbm(double& outDbm)
{
    uint8_t rssiValue = 0;
    if (!this->peekRegister(REG_RSSI_VALUE, rssiValue))
    {
        return false;
    }
    outDbm = -static_cast<double>(rssiValue) / 2.0; // RegRssiValue -> dBm, see the datasheet's RSSI section
    return true;
}

// Runs on a dedicated task for the driver's lifetime, invoking the user callback for every
// decoded frame. Kept off the ISR entirely: the callback reaches BlindController/esp_matter,
// neither of which is ISR-safe.
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

// MCPWM capture event callback for DIO2: cap_value/cap_edge are latched atomically by hardware at
// the instant of the electrical edge, so this can't misreport an edge's time/level even if the
// callback itself is delayed.
bool IRAM_ATTR SX1278Driver::onDio2Capture(mcpwm_cap_channel_handle_t /*capChannel*/,
                                            const mcpwm_capture_event_data_t* edata, void* userCtx)
{
    auto* self = static_cast<SX1278Driver*>(userCtx);
    int level = (edata->cap_edge == MCPWM_CAP_EDGE_POS) ? 1 : 0; // POS == rising == pin now high
    return self->handleReceivedEdge(edata->cap_value, level);
}

// Applies one confirmed (non-glitch) edge to rxState, returning true with outCompletedFrame filled
// in if it completes a 40-bit frame. Caller must hold rxStateMux; runs from both ISR context
// (handleReceivedEdge) and task context (noiseTimeoutCallback), so it touches nothing but its
// arguments and rxState.
//
// Mirrors buildWaveform()'s encoding: a HIGH ("mark") run in the SYNC_US range moves to
// AwaitingSyncGap; the LOW ("space") gap that follows is checked against RX_POST_SYNC_GAP_*_US
// before frame collection starts (a second independent coincidence, to tell a real sync pulse
// apart from a noise glitch); every mark after that decodes one payload bit by duration alone.
// Anything that doesn't fit an expected width abandons the in-progress frame and waits for the
// next sync pulse, so a corrupted reception self-heals on the next button-press repeat.
bool IRAM_ATTR SX1278Driver::applyEdgeToState(uint32_t edgeTicks, int level, std::array<uint8_t, 5>& outCompletedFrame)
{
    bool frameComplete = false;
    // Wraparound-safe: both operands are the same wrapping 32-bit tick counter, so plain unsigned
    // subtraction is correct across a wrap; only convert to microseconds *after* subtracting (see
    // rxCaptureTicksPerUs in the header).
    uint32_t deltaTicks = static_cast<uint32_t>(edgeTicks - this->rxState.lastEdgeTicks);
    this->rxState.lastEdgeTicks = edgeTicks;
    this->rxState.lastEdgeLevel = level;
    int64_t durationUs = deltaTicks / this->rxCaptureTicksPerUs;

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
            else if (durationUs >= RX_LONG_MIN_US && durationUs <= RX_LONG_MAX_US)
            {
                bit = true;
            }
            else
            {
                // Dead zone (or wildly out of range): too ambiguous to guess, so abandon this frame.
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
        // Rising edge: the LOW ("space") gap right after the sync mark was durationUs long.
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

    return frameComplete;
}

// Runs in ISR context on every DIO2 edge while the radio idles in receive mode (disabled during
// transmission). Never applies an edge to rxState directly, since a glitch looks identical to a
// real edge until the *next* edge shows how long the run it opened lasted -- instead buffers the
// latest edge in rxPendingValid/rxPendingEdgeTicks/rxPendingLevel:
//   - next edge < RX_NOISE_MAX_US later: both edges are noise, buffer is cleared;
//   - next edge >= RX_NOISE_MAX_US later: the buffered edge is applied, this one takes its place;
//   - no next edge at all (last edge of a burst): rxNoiseTimer's timeout applies it instead.
// Returns whether sending the completed-frame queue item woke a higher-priority task; the MCPWM
// driver's ISR trampoline calls portYIELD_FROM_ISR() itself based on this, so we must not.
bool IRAM_ATTR SX1278Driver::handleReceivedEdge(uint32_t nowTicks, int level)
{
    bool frameComplete = false;
    std::array<uint8_t, 5> completedFrame{};

    portENTER_CRITICAL_ISR(&this->rxStateMux);

    // Spurious duplicate dispatch: MCPWM's shared-interrupt handler can re-fire this callback with
    // byte-for-byte identical cap_value/cap_edge if its status-bit clear hasn't yet propagated when
    // the vector re-triggers, even though no new electrical edge happened. 
    // Bail out before touching rxIsrEdgeCount/PCNT at all.
    if (this->rxLastReceivedLevel != -1 && nowTicks == this->rxLastReceivedTicks && level == this->rxLastReceivedLevel)
    {
        portEXIT_CRITICAL_ISR(&this->rxStateMux);
        return false;
    }
    this->rxLastReceivedTicks = nowTicks;
    this->rxLastReceivedLevel = level;

    ++this->rxIsrEdgeCount;
    // Set when the PCNT reconciliation below fully accounts for this edge
    bool edgeRecoveredByPcnt = false;
    if (this->rxEdgeCountUnit != nullptr)
    {
        int pcntCount = 0;
        pcnt_unit_get_count(this->rxEdgeCountUnit, &pcntCount);
        if (pcntCount != this->rxIsrEdgeCount)
        {
            bool haveReferenceLevel = this->rxPendingValid || this->rxState.lastEdgeLevel != -1;
            int referenceLevel = this->rxPendingValid ? this->rxPendingLevel : this->rxState.lastEdgeLevel;
            if (pcntCount == this->rxIsrEdgeCount + 1 && haveReferenceLevel && referenceLevel == level)
            {
                // Common case: exactly one edge is missing -- a short glitch  raced MCPWM's single latch 
                // and overwrote it before the ISR serviced it. 
                // Since DIO2 can only alternate level and this edge matches the reference level, the
                // missing edge must have been the opposite level: together they're a glitch pair
                // that returns to the reference level, the same shape RX_NOISE_MAX_US already
                // discards below. Recover by leaving rxState/the pending buffer untouched
                edgeRecoveredByPcnt = true;
            }
            else
            {
                // MCPWM capture never fired for at least one real DIO2 edge. Whatever is buffered in
                // rxPendingValid straddles the gap and was measured against a stale reference point,
                // so discard it and abandon any in-progress frame; decoding resyncs on the next sync
                // pulse, same as the BitInvalid/GapInvalid paths in applyEdgeToState().
                this->rxState.phase = RxState::Phase::Idle;
                this->rxPendingValid = false;
                esp_timer_stop(this->rxNoiseTimer);
            }
            this->rxIsrEdgeCount = pcntCount; // re-baseline to hardware ground truth
        }

        // rxIsrEdgeCount now matches the live PCNT count read above -- periodically reset both back
        // to 0 together, well before either nears PCNT_HIGH_LIMIT.
        if (this->rxIsrEdgeCount >= PCNT_RESET_THRESHOLD)
        {
            pcnt_unit_clear_count(this->rxEdgeCountUnit);
            this->rxIsrEdgeCount = 0;
        }
    }

    if (!edgeRecoveredByPcnt)
    {
        // Wraparound-safe unsigned subtraction on raw ticks, converted to microseconds only after
        // subtracting -- see rxCaptureTicksPerUs in the header.
        uint32_t glitchDeltaUs = static_cast<uint32_t>(nowTicks - this->rxPendingEdgeTicks) / this->rxCaptureTicksPerUs;
        if (this->rxPendingValid && glitchDeltaUs < RX_NOISE_MAX_US)
        {
            // Glitch: the buffered edge and this one are both noise.
            esp_timer_stop(this->rxNoiseTimer);
            this->rxPendingValid = false;
        }
        else
        {
            if (this->rxPendingValid)
            {
                frameComplete =
                    this->applyEdgeToState(this->rxPendingEdgeTicks, this->rxPendingLevel, completedFrame);
            }

            esp_timer_stop(this->rxNoiseTimer);

            this->rxPendingValid = true;
            this->rxPendingEdgeTicks = nowTicks;
            this->rxPendingLevel = level;
            esp_timer_start_once(this->rxNoiseTimer, RX_NOISE_MAX_US);
        }
    }

    portEXIT_CRITICAL_ISR(&this->rxStateMux);

    bool higherPriorityTaskWoken = false;
    if (frameComplete)
    {
        BaseType_t woken = pdFALSE;
        xQueueSendFromISR(this->rxFrameQueue, &completedFrame, &woken);
        higherPriorityTaskWoken = (woken == pdTRUE);
    }
    return higherPriorityTaskWoken;
}

// Runs on the esp_timer task, RX_NOISE_MAX_US after the last edge handleReceivedEdge() buffered,
// unless a closer or later edge already cleared rxPendingValid. Applies that edge the same way a
// confirming next edge would, so the very last edge of a burst still makes it into rxState.
void SX1278Driver::noiseTimeoutCallback(void* arg)
{
    auto* self = static_cast<SX1278Driver*>(arg);
    bool frameComplete = false;
    std::array<uint8_t, 5> completedFrame{};

    portENTER_CRITICAL(&self->rxStateMux);
    if (self->rxPendingValid)
    {
        frameComplete = self->applyEdgeToState(self->rxPendingEdgeTicks, self->rxPendingLevel, completedFrame);
        self->rxPendingValid = false;
    }
    portEXIT_CRITICAL(&self->rxStateMux);

    if (frameComplete)
    {
        xQueueSend(self->rxFrameQueue, &completedFrame, 0);
    }
}

// Runs on a dedicated task for the driver's lifetime, transmitting queued commands strictly in
// submission order. Whenever it isn't transmitting, it leaves the radio in continuous receive mode
// listening for the physical remote (see enterReceiveMode()).
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
    this->writeRegister(REG_OP_MODE, OPMODE_STANDBY);

    size_t total = waveform.size();

    this->writeRegister(REG_PACKET_CONFIG2, PACKET_CONFIG2_DATA_MODE_PACKET | static_cast<uint8_t>((total >> 8) & 0x07));
    this->writeRegister(REG_PAYLOAD_LENGTH, static_cast<uint8_t>(total & 0xFF));

    // Pre-fill the FIFO before enabling Tx, per the datasheet's "Handling Large Packets" sequence.
    size_t sent = 0;
    size_t preload = std::min(total, FIFO_CAPACITY);
    while (sent < preload)
    {
        size_t chunk = std::min(preload - sent, FIFO_CHUNK);
        this->writeFifo(waveform.data() + sent, chunk);
        sent += chunk;
    }

    this->writeRegister(REG_OP_MODE, OPMODE_TX);

    int64_t deadline = esp_timer_get_time() + FIFO_WAIT_TIMEOUT_US;
    while (sent < total)
    {
        if (gpio_get_level(SX1278_DIO1_GPIO) == 0) // FifoLevel cleared: level has drained to the threshold
        {
            size_t chunk = std::min(total - sent, FIFO_CHUNK);
            this->writeFifo(waveform.data() + sent, chunk);
            sent += chunk;
            deadline = esp_timer_get_time() + FIFO_WAIT_TIMEOUT_US;
        }
        else if (esp_timer_get_time() > deadline)
        {
            ESP_LOGE(TAG, "Timed out refilling SX1278 FIFO, aborting transmission");
            this->writeRegister(REG_OP_MODE, OPMODE_STANDBY);
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

    this->writeRegister(REG_OP_MODE, OPMODE_STANDBY);
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

    // Stop reacting to DIO2 edges and drop out of continuous Rx before touching any packet-framing
    // registers, and cancel any RX edge still waiting out RX_NOISE_MAX_US.
    esp_err_t err = mcpwm_capture_channel_disable(this->rxCaptureChannel);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to disable DIO2 capture channel: %d", err);
    }
    esp_timer_stop(this->rxNoiseTimer);
    portENTER_CRITICAL(&this->rxStateMux);
    this->rxPendingValid = false;
    this->rxLastReceivedLevel = -1;
    portEXIT_CRITICAL(&this->rxStateMux);

    std::vector<uint8_t> waveform = buildWaveform(data);

    for (uint8_t i = 0; i < repeats; ++i)
    {
        this->transmitWaveform(waveform);
        if (i + 1 < repeats)
        {
            esp_rom_delay_us(RESET_US);
        }
    }
}
