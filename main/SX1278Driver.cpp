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

// dio2SetupTask() runs configureEdgeCounter()/configureDioPins() from core 1 instead of directly
// from init() (core 0), so the MCPWM capture and PCNT interrupts esp_intr_alloc() registers inside
// those calls bind to core 1 -- away from WiFi/BLE's own interrupt dispatch, both pinned to core 0
// (see the comment in configureDioPins() this was written to address).
constexpr uint32_t DIO2_SETUP_TASK_STACK_WORDS = 3072;
constexpr UBaseType_t DIO2_SETUP_TASK_PRIORITY = 5;
constexpr BaseType_t DIO2_SETUP_CORE = 1;

// Passed into dio2SetupTask() as its FreeRTOS task argument.
struct Dio2SetupContext
{
    SX1278Driver* driver;
    SemaphoreHandle_t doneSemaphore;
};

// ESP32's PCNT counter register is a signed 16-bit field.
constexpr int PCNT_LOW_LIMIT = -32768;
constexpr int PCNT_HIGH_LIMIT = 32767;
// handleReceivedEdge() resets rxIsrEdgeCount (and the PCNT hardware counter alongside it) back to 0
// once it reaches this, well before either could approach the hardware ceiling above -- see there.
// Plain threshold, not a hardware watch point, so there's no exact-value-match concern; any value
// comfortably below PCNT_HIGH_LIMIT is safe.
constexpr int PCNT_RESET_THRESHOLD = 20000;

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

// Sets up the PCNT ground-truth edge counter for DIO2 -- see the header comment on rxEdgeCountUnit.
// Must run before configureDioPins(): pcnt_new_channel() unconditionally force-enables DIO2's
// internal pull-up (its own doc comment: "input mode with pull up enabled"), and configureDioPins()'s
// mcpwm_new_capture_channel() call explicitly disables it again (pull_up=false, matching
// GPIO_PULLUP_DISABLE elsewhere) -- so that disable must be the one that runs last.
void SX1278Driver::configureEdgeCounter()
{
    pcnt_unit_config_t unitConfig = {};
    unitConfig.low_limit = PCNT_LOW_LIMIT;
    unitConfig.high_limit = PCNT_HIGH_LIMIT;
    // No intr_priority to set: this unit registers no watch point/event callback below, so it never
    // allocates an interrupt at all -- handleReceivedEdge() only ever polls its count directly.
    esp_err_t err = pcnt_new_unit(&unitConfig, &this->rxEdgeCountUnit);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create DIO2 edge-count PCNT unit: %d", err);
        return;
    }

    // No glitch filter: this counter must match MCPWM capture's raw per-ISR-call count 1:1,
    // including the noise glitches handleReceivedEdge()'s own buffering later filters out.
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
    // Deliberately not calling pcnt_unit_start() here -- enterReceiveMode()/transmitWaveform() start
    // and stop it in lockstep with the MCPWM capture channel, same pattern as
    // mcpwm_capture_channel_enable()/disable().
}

// Runs configureEdgeCounter()/configureDioPins() from a task pinned to core 1 instead of directly
// from init() (which runs on core 0, see app_main.cpp), so the interrupts esp_intr_alloc() registers
// inside those two calls bind to core 1 instead of contending with WiFi/BLE's own interrupt dispatch
// on core 0 -- see the DIO2_SETUP_* comment above. init() blocks on doneSemaphore so it stays
// synchronous from its caller's perspective; this task deletes itself once done.
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
    // enterReceiveMode()) this pin mirrors the raw OOK-demodulated envelope in real time. Every
    // edge is timestamped in hardware by the MCPWM capture peripheral -- the tick count and edge
    // polarity are latched atomically at the instant of the electrical edge, so the timestamp
    // handleReceivedEdge() sees can't be corrupted by ISR scheduling delay the way a software
    // esp_timer_get_time() read from inside a GPIO ISR could be (WiFi/BLE, both pinned to core 0
    // alongside this driver's own init, run higher-priority interrupts that can preempt a level-1
    // GPIO ISR for long enough to badly misreport an edge's time -- this used to produce spurious
    // multi-millisecond gaps that broke frame decoding). The capture channel starts disabled since
    // DIO2's meaning while transmitting/mid-configuration is undefined; enterReceiveMode() is what
    // turns it on.
    mcpwm_capture_timer_config_t capTimerConfig = {};
    capTimerConfig.group_id = 0;
    capTimerConfig.clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
    // Best-effort hint only: on plain ESP32 the capture timer's clock is hardwired to APB and this
    // is silently ignored (see the rxCaptureTicksPerUs comment in the header), so don't assume it
    // took effect -- the actual resolution is read back just below instead.
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
    // pull_up/pull_down left false: matches GPIO_PULLUP_DISABLE/GPIO_PULLDOWN_DISABLE above.
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
    // Deliberately not calling mcpwm_capture_channel_enable() here -- a freshly created channel
    // starts disabled, same as gpio_intr_disable() used to; enterReceiveMode() enables it.
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
    writeRegister(REG_OOK_FIX, 0x60);

    // AgcAutoOn=1, RxTrigger=001 (Rssi interrupt): the LNA gain (re-)converges whenever RSSI
    // crosses the threshold, i.e. whenever a transmission begins after a silent gap -- exactly
    // the pattern a burst of remote button presses produces during otherwise-idle continuous Rx.
    writeRegister(REG_RX_CONFIG, 0x09);
}

void SX1278Driver::init()
{
    resetChip();

    // See dio2SetupTask()'s comment: runs configureEdgeCounter()/configureDioPins() from core 1 so
    // their interrupts bind there instead of core 0. This blocks until that task signals completion,
    // so init() remains synchronous from app_main.cpp's perspective.
    SemaphoreHandle_t dio2SetupDone = xSemaphoreCreateBinary();
    Dio2SetupContext dio2SetupContext{ this, dio2SetupDone };
    xTaskCreatePinnedToCore(&SX1278Driver::dio2SetupTask, "sx1278_dio2_setup", DIO2_SETUP_TASK_STACK_WORDS,
                             &dio2SetupContext, DIO2_SETUP_TASK_PRIORITY, nullptr, DIO2_SETUP_CORE);
    xSemaphoreTake(dio2SetupDone, portMAX_DELAY);
    vSemaphoreDelete(dio2SetupDone);

    configureDebugPins();

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

// MCPWM capture event callback for DIO2: cap_value/cap_edge are latched atomically by hardware at
// the instant of the electrical edge (see the comment in configureDioPins()), so unlike the old
// GPIO-ISR approach this can't misreport an edge's time or level even if this callback itself is
// delayed in being serviced.
bool IRAM_ATTR SX1278Driver::onDio2Capture(mcpwm_cap_channel_handle_t /*capChannel*/,
                                            const mcpwm_capture_event_data_t* edata, void* userCtx)
{
    auto* self = static_cast<SX1278Driver*>(userCtx);
    int level = (edata->cap_edge == MCPWM_CAP_EDGE_POS) ? 1 : 0; // POS == rising == pin now high
    return self->handleReceivedEdge(edata->cap_value, level);
}

// Applies one confirmed (not a noise glitch) edge to rxState and returns true, with
// outCompletedFrame filled in, if it completes a 40-bit frame; Caller must hold rxStateMux; 
// runs both from ISR context (handleReceivedEdge) and task context (noiseTimeoutCallback), 
// so it touches nothing but its arguments and rxState.
//
// Reconstructs the same 40-bit Dooya frame buildWaveform() encodes: a HIGH ("mark") run in the
// SYNC_US range moves to AwaitingSyncGap, the LOW ("space") run that follows it is checked against
// RX_POST_SYNC_GAP_*_US before frame collection actually starts (this second, independent
// coincidence is what tells a real sync pulse apart from a noise glitch that happens to land in
// the SYNC_US window), and every mark after that decodes one payload bit by its duration alone
// (see the RX_*_US tolerance comment above). Anything that doesn't fit an expected width abandons
// the in-progress frame and waits for the next sync pulse, so a corrupted/partial reception
// self-heals on the next button-press repeat instead of needing an explicit timeout.
bool IRAM_ATTR SX1278Driver::applyEdgeToState(uint32_t edgeTicks, int level, std::array<uint8_t, 5>& outCompletedFrame)
{
    bool frameComplete = false;
    // Wraparound-safe: both operands are the same wrapping 32-bit raw tick counter, so plain
    // unsigned subtraction gives the correct elapsed ticks even across a wrap -- only convert to
    // microseconds (via rxCaptureTicksPerUs) *after* subtracting; converting each absolute tick
    // value first would wrap at the wrong modulus (see the header comment on rxCaptureTicksPerUs).
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
                // Falls in a dead zone (or wildly out of range): too ambiguous to guess at, so
                // abandon this frame rather than risk committing a wrong bit. Self-heals on the
                // next sync pulse -- either a repeat of the same button press or the next one.
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

    return frameComplete;
}

// Runs in ISR context (the MCPWM capture channel's callback) on every DIO2 edge while the radio is
// idling in receive mode (disabled during transmission, see transmitWaveform()). Never applies an
// edge to rxState directly --
// a noise glitch looks identical to a real edge until the *next* edge shows how long the run it
// opened actually lasted, so this only ever buffers the latest edge in rxPendingValid/
// rxPendingEdgeTicks/rxPendingLevel and lets applyEdgeToState() see it once it's confirmed real:
//   - if the next edge arrives under RX_NOISE_MAX_US later, the buffered edge and this new one are
//     both noise -- neither is ever applied, and the buffer is simply cleared;
//   - if the next edge arrives RX_NOISE_MAX_US or later, the buffered edge survived long enough to
//     be real -- it's applied now, and this new edge takes its place in the buffer;
//   - if no next edge arrives at all (the last edge of a burst), rxNoiseTimer's timeout applies it
//     from noiseTimeoutCallback instead, once RX_NOISE_MAX_US has passed with nothing to glitch it.
// Returns whether sending the completed-frame queue item woke a higher-priority task, per the
// mcpwm_capture_event_cb_t contract onDio2Capture() forwards this into: the MCPWM driver's ISR
// trampoline calls portYIELD_FROM_ISR() itself based on that return value, so this function must
// not (and no longer does) call it directly.
bool IRAM_ATTR SX1278Driver::handleReceivedEdge(uint32_t nowTicks, int level)
{
    //gpio_set_level(SX1278_RX_DEBUG_1_GPIO, level);
    //gpio_set_level(SX1278_RX_DEBUG_2_GPIO, 1);

    bool frameComplete = false;
    std::array<uint8_t, 5> completedFrame{};

    portENTER_CRITICAL_ISR(&this->rxStateMux);

    // Spurious duplicate dispatch: MCPWM's capture interrupt is registered via
    // esp_intr_alloc_intrstatus's shared-interrupt mechanism, which re-reads the group's raw status
    // register fresh every time the underlying interrupt vector fires (see shared_intr_isr() in
    // esp-idf's intr_alloc.c). mcpwm_capture_default_isr() clears its status bit with a plain write
    // and only then reads the separate cap_value/cap_edge latch registers -- if that clear hasn't
    // yet propagated through to the status register by the time the vector re-triggers (more likely
    // under the tight back-to-back timing a real glitch produces), the still-set bit causes a
    // second, spurious call to this function with byte-for-byte identical cap_value/cap_edge, since
    // no new electrical edge actually happened. A genuine edge can, for all practical purposes,
    // never land on the exact same capture-timer tick as the previous one, so comparing nowTicks
    // against rxLastReceivedTicks -- the very last edge this function was actually called with, kept
    // up to date regardless of whether that edge was applied, buffered, or discarded as a glitch --
    // reliably tells the two apart. (rxPendingEdgeTicks/rxState.lastEdgeTicks are NOT substitutes for
    // this: a glitch-cancelled edge updates neither, so right after a glitch is skipped they'd still
    // point at whatever edge came before it, missing a duplicate of the just-cancelled one entirely.)
    // Bail out before touching rxIsrEdgeCount/PCNT at all, so the duplicate can't masquerade as a
    // hardware-dropped edge (the spurious negative-delta HardwareEdgeDropped this was producing).
    if (this->rxLastReceivedLevel != -1 && nowTicks == this->rxLastReceivedTicks && level == this->rxLastReceivedLevel)
    {
        portEXIT_CRITICAL_ISR(&this->rxStateMux);
        //gpio_set_level(SX1278_RX_DEBUG_2_GPIO, 0);
        return false;
    }
    this->rxLastReceivedTicks = nowTicks;
    this->rxLastReceivedLevel = level;

    ++this->rxIsrEdgeCount;
    // Set when the PCNT reconciliation below fully accounts for this edge itself (see the
    // HardwareEdgeRecovered case) -- the normal glitch/apply logic further down must not also run
    // for it, since there's nothing left to buffer or apply: the recovery already decided this
    // edge's fate.
    bool edgeRecoveredByPcnt = false;
    if (this->rxEdgeCountUnit != nullptr)
    {
        int pcntCount = 0;
        pcnt_unit_get_count(this->rxEdgeCountUnit, &pcntCount);
        if (pcntCount != this->rxIsrEdgeCount)
        {
            // The edge to compare this one's level against: the still-unconfirmed pending edge if
            // there is one, otherwise the last edge applyEdgeToState() actually confirmed -- covers
            // both a glitch racing MCPWM's latch shortly after a real edge (rxPendingValid still
            // true) *and* one arriving after that pending edge already got applied by rxNoiseTimer,
            // or cancelled outright as its own glitch (rxPendingValid false either way, but
            // rxState.lastEdgeLevel still holds the DIO2 line's actual last known-good level). No
            // reference exists yet only right after enterReceiveMode()/transmitWaveform() resets
            // rxState, when lastEdgeLevel's sentinel -1 can't match either real level.
            bool haveReferenceLevel = this->rxPendingValid || this->rxState.lastEdgeLevel != -1;
            int referenceLevel = this->rxPendingValid ? this->rxPendingLevel : this->rxState.lastEdgeLevel;
            if (pcntCount == this->rxIsrEdgeCount + 1 && haveReferenceLevel && referenceLevel == level)
            {
                // The common case: a very short glitch shortly after a real edge raced the MCPWM
                // capture channel's single latch and overwrote it before the ISR serviced it (see
                // configureEdgeCounter()) -- exactly one edge is missing. Because DIO2 can only ever
                // alternate level, and this edge's level matches the reference edge above, the
                // missing edge must have had the opposite level: together they're a glitch pair that
                // returns to the reference level, exactly the shape RX_NOISE_MAX_US already discards
                // below. Recover by treating this edge as that glitch pair's second half -- leave
                // rxState/the pending buffer exactly as they are (as if neither the missing edge nor
                // this one ever happened) instead of aborting the whole in-progress frame.
                edgeRecoveredByPcnt = true;
            }
            else
            {
                // MCPWM capture never fired for at least one real DIO2 edge. Whatever is buffered in
                // rxPendingValid straddles the gap -- its duration was measured against a stale
                // reference point, so discard it and abandon any in-progress frame instead of letting
                // a corrupted duration reach applyEdgeToState(); decoding resyncs cleanly on the next
                // sync pulse, same as the BitInvalid/GapInvalid paths there.
                this->rxState.phase = RxState::Phase::Idle;
                this->rxPendingValid = false;
                esp_timer_stop(this->rxNoiseTimer);
            }
            this->rxIsrEdgeCount = pcntCount; // re-baseline to hardware ground truth
        }

        // rxIsrEdgeCount is now guaranteed equal to the live PCNT count read above (either they
        // already matched, or the re-baseline just above made them match) -- periodically reset both
        // back to 0 together, right here in the same edge that's already touching both, well before
        // either could approach PCNT_HIGH_LIMIT. No separate watch point/interrupt needed for this:
        // every edge already polls the live count, so a plain threshold check on it is sufficient.
        if (this->rxIsrEdgeCount >= PCNT_RESET_THRESHOLD)
        {
            pcnt_unit_clear_count(this->rxEdgeCountUnit);
            this->rxIsrEdgeCount = 0;
        }
    }

    if (!edgeRecoveredByPcnt)
    {
        // Wraparound-safe unsigned subtraction on raw ticks, converted to microseconds only after
        // subtracting -- see the comment on rxCaptureTicksPerUs in the header and in applyEdgeToState().
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

            // Stopped unconditionally, not just when rxPendingValid was set: noiseTimeoutCallback's
            // timer removes itself from esp_timer's internal alarm list (making it look "unarmed") and
            // drops esp_timer's own lock *before* it ever touches rxStateMux -- see esp_timer.c's
            // timer_process_alarm(). So a stale callback can still be in flight, about to apply/clear
            // whatever is *currently* pending, even after we've moved on to a newer edge here. If that
            // happens, the fresh timer we armed for this newer edge gets orphaned: still ticking, but
            // for an edge the stale callback already consumed. Skipping this stop() whenever our own
            // rxPendingValid happened to read false left esp_timer_start_once() below silently fail
            // (ESP_ERR_INVALID_STATE, unchecked) on that orphaned timer, so the edge being buffered here
            // got no flush timer at all -- harmless for most edges (the next edge's arrival still
            // applies it via the branch above), but fatal for the last mark of a frame, which has no
            // next edge to rescue it before the next sync pulse's edge shows up and gets misread against
            // a stale reference point. Calling stop() here regardless -- a no-op if nothing was armed --
            // guarantees the start_once() below is never rejected as "already armed".
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
    //gpio_set_level(SX1278_RX_DEBUG_2_GPIO, 0);
    return higherPriorityTaskWoken;
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
