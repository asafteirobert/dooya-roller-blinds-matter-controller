#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

// mcpwm_capture_event_data_t is an anonymous-struct typedef, so it can't be forward-declared.
#include <driver/mcpwm_types.h>

// Forward-declared instead of including the full drivers, since these handles are just pointers.
struct spi_device_t;
struct esp_timer;
struct pcnt_unit_t;
struct pcnt_chan_t;

// Drives an SX1278 module over VSPI to replay Dooya remote control commands, and to decode the
// same commands when the physical remote transmits them directly to the blind.
//
// The chip has no built-in Dooya support, so in both directions the 40-bit command is hand-coded
// against the exact OOK/PWM pulse train the remote and motor speak, rather than relying on the
// SX1278's packet engine (which only recognises framing this protocol doesn't have).
//
// - send() only enqueues the command and returns immediately; a background task owns the SPI
//   device and transmits queued commands in order, so callers never block on air time.
// - Whenever that task isn't transmitting, it leaves the radio in continuous OOK receive mode.
//   DIO2 mirrors the demodulated envelope; an MCPWM capture channel timestamps every edge in
//   hardware and software decodes it, handing completed frames to a receiver task which invokes
//   the callback registered via setReceiveCallback().
class SX1278Driver
{
    static constexpr char *TAG = "SX1278Driver";
public:
    // Invoked from a dedicated background task (never ISR context) for every complete 40-bit
    // frame decoded off the air while the radio is idling in receive mode.
    using ReceiveCallback = std::function<void(const std::array<uint8_t, 5>& data)>;

    void init();
    void send(const std::array<uint8_t, 5>& data, uint8_t repeats = 1);
    void setReceiveCallback(ReceiveCallback callback);

    // Diagnostic register access for the "sx1278reg"/"sx1278rssi" console commands (see
    // CliCommands.cpp). RegOokFix (see configureReceiver()) is board/environment-specific and
    // needs live tuning. Safe to call concurrently with normal operation. Returns false without
    // touching the bus if the driver never finished initialising.
    bool peekRegister(uint8_t address, uint8_t& outValue);
    bool pokeRegister(uint8_t address, uint8_t value);
    bool peekRssiDbm(double& outDbm);

private:
    struct QueuedCommand
    {
        std::array<uint8_t, 5> data;
        uint8_t repeats;
    };

    void resetChip();
    static void dio2SetupTask(void* arg);
    void configureEdgeCounter();
    void configureDioPins();
    void configureDebugPins();
    void configureRadio();
    void configureReceiver();
    void transmitWaveform(const std::vector<uint8_t>& waveform);
    void sendNow(const std::array<uint8_t, 5>& data, uint8_t repeats);
    void enterReceiveMode();

    void writeRegister(uint8_t address, uint8_t value);
    uint8_t readRegister(uint8_t address);
    void writeFifo(const uint8_t* data, size_t length);

    static void senderTask(void* arg);
    static void receiverTask(void* arg);
    static bool onDio2Capture(mcpwm_cap_channel_handle_t capChannel, const mcpwm_capture_event_data_t* edata,
                               void* userCtx);
    bool handleReceivedEdge(uint32_t nowTicks, int level);
    bool applyEdgeToState(uint32_t edgeTicks, int level, std::array<uint8_t, 5>& outCompletedFrame);
    static void noiseTimeoutCallback(void* arg);

    spi_device_t* spiDevice = nullptr;
    QueueHandle_t commandQueue = nullptr;
    TaskHandle_t senderTaskHandle = nullptr;

    ReceiveCallback receiveCallback;
    QueueHandle_t rxFrameQueue = nullptr;
    TaskHandle_t receiverTaskHandle = nullptr;

    // In-progress RX decode. Mutated only by applyEdgeToState(), guarded by rxStateMux.
    struct RxState
    {
        enum class Phase : uint8_t
        {
            Idle,            // waiting for a sync-length mark
            AwaitingSyncGap, // saw a sync mark, waiting to confirm the low gap that must follow it
            InFrame,         // sync confirmed, collecting the 40 payload bits
        };
        Phase phase = Phase::Idle;
        uint8_t bitCount = 0;
        std::array<uint8_t, 5> frame{};
        uint32_t lastEdgeTicks = 0; // raw MCPWM capture-timer ticks -- see rxCaptureTicksPerUs
        // DIO2's last known-good confirmed level (-1 = none yet). Distinct from handleReceivedEdge's
        // rxPendingLevel, which tracks the latest edge seen even if still unconfirmed or a glitch --
        // this is the reference level hardware-drop recovery needs once no pending edge remains.
        int lastEdgeLevel = -1;
    } rxState;

    // An edge seen but not yet believed: a genuine glitch looks identical to a real edge until the
    // *next* edge shows how long the run it opened actually lasted. Applied to rxState once
    // RX_NOISE_MAX_US passes without a glitch-close, either via the next edge or rxNoiseTimer.
    bool rxPendingValid = false;
    uint32_t rxPendingEdgeTicks = 0; // raw MCPWM capture-timer ticks -- see rxCaptureTicksPerUs
    int rxPendingLevel = 0;
    struct esp_timer* rxNoiseTimer = nullptr;

    // The very last edge received from the MCPWM capture ISR regardless of outcome (applied,
    // buffered, or discarded as a glitch). Used only to detect a spurious duplicate redispatch of
    // the same hardware capture event -- see handleReceivedEdge().
    uint32_t rxLastReceivedTicks = 0;
    int rxLastReceivedLevel = -1;

    // MCPWM capture channel for DIO2: latches (tick count, edge polarity) atomically in hardware,
    // immune to ISR scheduling jitter that a software timestamp would pick up.
    mcpwm_cap_timer_handle_t rxCaptureTimer = nullptr;
    mcpwm_cap_channel_handle_t rxCaptureChannel = nullptr;
    // Actual resolution read back after creating the timer (ESP32's capture clock is hardwired to
    // 80MHz APB, so the resolution_hz hint in configureDioPins() is ignored). Ticks are divided by
    // this only *after* a wraparound-safe raw subtraction, never before -- pre-dividing would wrap
    // at the wrong modulus (see applyEdgeToState()/handleReceivedEdge()).
    uint32_t rxCaptureTicksPerUs = 1;

    // Ground-truth edge counter for DIO2 (see configureEdgeCounter()): PCNT increments straight
    // from GPIO transitions in hardware with no interrupt of its own, so unlike the single-latch
    // MCPWM channel it can't silently lose a count. handleReceivedEdge() polls it on every edge
    // against its own tally (rxIsrEdgeCount) to detect coalesced edges.
    pcnt_unit_t* rxEdgeCountUnit = nullptr;
    pcnt_chan_t* rxEdgeCountChannel = nullptr;
    // Per-ISR-call edge count (glitches included), periodically reset alongside the PCNT hardware
    // counter well before either nears the 16-bit register's ceiling -- see PCNT_RESET_THRESHOLD.
    // Touched only under rxStateMux, only from handleReceivedEdge (ISR context).
    int32_t rxIsrEdgeCount = 0;

    portMUX_TYPE rxStateMux = portMUX_INITIALIZER_UNLOCKED;
};
