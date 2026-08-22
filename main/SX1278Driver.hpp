#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

// Lightweight types-only header (just the capture handle typedefs and mcpwm_capture_event_data_t)
// -- unlike spi_device_t/esp_timer below, mcpwm_capture_event_data_t is an anonymous-struct
// typedef with no tag name, so it can't be forward-declared and this can't be deferred to the .cpp.
#include <driver/mcpwm_types.h>

// Avoids pulling driver/spi_master.h (and its transitive ESP-IDF includes) into every file that
// includes this header; spi_device_handle_t is just a pointer to this struct.
struct spi_device_t;
// Same trick for esp_timer.h; esp_timer_handle_t is just a pointer to this struct.
struct esp_timer;
// Same trick for driver/pulse_cnt.h; pcnt_unit_handle_t/pcnt_channel_handle_t are just pointers to
// these structs -- unlike mcpwm_capture_event_data_t above, PCNT's only anonymous-struct-typedef
// (pcnt_watch_event_data_t) was solely for a watch-point callback signature this driver no longer
// registers (see configureEdgeCounter()), so nothing here needs the full header any more.
struct pcnt_unit_t;
struct pcnt_chan_t;

// Drives an SX1278 module over VSPI to replay Dooya remote control commands, and to decode the
// same commands when the physical remote transmits them directly to the blind.
//
// The chip has no built-in Dooya support, so in both directions the 40-bit command is hand-coded
// against the exact OOK/PWM pulse train the remote and motor speak, rather than relying on the
// SX1278's packet engine (which only recognises framing this protocol doesn't have: no
// preamble/sync word/CRC).
//
// - send() only enqueues the command and returns immediately; a dedicated background task owns
//   the SPI device and transmits queued commands one at a time, in the order they were
//   submitted, so callers (Matter/timer callbacks) never block on the air time of a transmission.
// - Whenever that task isn't actively transmitting, it leaves the radio in continuous OOK
//   receive mode, listening for the physical remote. DIO2 mirrors the raw demodulated envelope
//   in that mode; an MCPWM capture channel timestamps every edge in hardware (immune to the
//   timing jitter a software GPIO-ISR timestamp would pick up from WiFi/BLE interrupt latency)
//   and software decodes it (mirroring the encoding transmitWaveform() produces), handing off
//   completed frames to a receiver task which invokes the callback registered via
//   setReceiveCallback().
class SX1278Driver
{
    static constexpr char *TAG = "SX1278Driver";
public:
    // Invoked, from a dedicated background task (never from ISR context), with every complete
    // 40-bit frame decoded off the air while the radio is idling in receive mode.
    using ReceiveCallback = std::function<void(const std::array<uint8_t, 5>& data)>;

    void init();
    void send(const std::array<uint8_t, 5>& data, uint8_t repeats = 1);
    void setReceiveCallback(ReceiveCallback callback);

    // Diagnostic register access, exposed for interactive RF tuning via the "sx1278reg"/
    // "sx1278rssi" console commands (see CliCommands.cpp). The OOK receive threshold in
    // particular (RegOokFix, see configureReceiver()) is board/environment-specific and can't be
    // gotten right from firmware defaults alone -- the datasheet's own tuning procedure is to
    // adjust it live and watch the result. Safe to call from any task, including concurrently
    // with normal operation: the underlying SPI transaction is serialized against senderTask's
    // own via the SPI driver's bus lock, though poking registers mid-transmission can still
    // disrupt that transmission. Returns false (without touching the bus) if the driver never
    // finished initialising.
    bool peekRegister(uint8_t address, uint8_t& outValue);
    bool pokeRegister(uint8_t address, uint8_t value);
    // Convenience for "sx1278rssi": instantaneous received signal strength while idling in
    // receive mode, in dBm.
    bool peekRssiDbm(double& outDbm);

private:
    struct QueuedCommand
    {
        std::array<uint8_t, 5> data;
        uint8_t repeats;
    };

    // Diagnostic-only description of one RX edge-processing outcome -- either what
    // applyEdgeToState() did with a confirmed edge, or a glitch handleReceivedEdge() discarded
    // before the edge ever reached applyEdgeToState(). It's an out-parameter rather than logged
    // directly at the point of detection because every caller runs under rxStateMux (ISR-disabled
    // critical section from handleReceivedEdge, interrupts-disabled critical section from
    // noiseTimeoutCallback) -- logging while holding that lock would stall the other core for
    // however long the print takes. Callers log it via logRxEvent() only after releasing the lock.
    struct RxEvent
    {
        enum class Kind : uint8_t
        {
            None,
            GlitchDropped,    // two edges arrived within RX_NOISE_MAX_US of each other while InFrame; both
                              // discarded as noise (only reported mid-frame -- see handleReceivedEdge)
            SyncDetected,     // a mark in the sync-length range was seen
            SyncAbortedFrame, // ...and it interrupted an already in-progress frame, dropping it
            GapInvalid,       // the post-sync low gap didn't match, dropped before any bits
            BitInvalid,       // a mark matched neither the short nor long band, dropped mid-frame
            HardwareEdgeDropped, // the PCNT ground-truth edge counter diverged from the MCPWM-capture-
                                  // driven count -- MCPWM's single-latch channel coalesced >=1 real
                                  // DIO2 edges into one ISR call (see configureEdgeCounter()); whatever
                                  // was in flight is abandoned and decoding resyncs on the next sync pulse
            HardwareEdgeRecovered, // same underlying cause as HardwareEdgeDropped, but exactly one edge
                                    // was missing and it formed a glitch pair with the still-pending
                                    // edge (same level) -- absorbed like a normal glitch instead of
                                    // aborting the in-progress frame; see handleReceivedEdge()
        };
        Kind kind = Kind::None;
        // For every other Kind this is a duration in microseconds; for HardwareEdgeDropped it's
        // repurposed as the raw edge-count delta (PCNT count minus software count) instead.
        int64_t durationUs = 0;
        uint8_t bitCount = 0; // bits collected so far, for GlitchDropped/AbortedFrame/BitInvalid
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
    bool applyEdgeToState(uint32_t edgeTicks, int level, std::array<uint8_t, 5>& outCompletedFrame, RxEvent& outEvent);
    static void noiseTimeoutCallback(void* arg);
    static void logRxEvent(const RxEvent& event);

    spi_device_t* spiDevice = nullptr;
    QueueHandle_t commandQueue = nullptr;
    TaskHandle_t senderTaskHandle = nullptr;

    ReceiveCallback receiveCallback;
    QueueHandle_t rxFrameQueue = nullptr;
    TaskHandle_t receiverTaskHandle = nullptr;

    // In-progress RX decode. Only ever mutated by applyEdgeToState() -- called either from
    // handleReceivedEdge (ISR context, DIO2 interrupt disabled while transmitting) once an edge is
    // confirmed real, or from noiseTimeoutCallback (esp_timer task context) -- and reset from
    // enterReceiveMode()/transmitWaveform() (task context, only while that interrupt is disabled);
    // rxStateMux makes that handover safe across cores.
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
        // Level of the most recent edge applyEdgeToState() actually confirmed, i.e. the DIO2 line's
        // last known-good settled level -- -1 means none yet this session (only true right after a
        // reset in enterReceiveMode()/transmitWaveform(), since RxState{}'s default already gives us
        // that for free). Deliberately *not* the same thing as handleReceivedEdge()'s rxPendingLevel:
        // that tracks the latest edge seen at all, even one still unconfirmed or one a glitch cancels
        // out entirely -- this tracks only edges that actually happened, which is what
        // handleReceivedEdge()'s hardware-drop recovery needs once rxPendingValid has gone false
        // (the pending edge got applied by rxNoiseTimer, or cancelled as a glitch) but a *further*
        // edge still needs a known-good reference level to recover against.
        int lastEdgeLevel = -1;
    } rxState;

    // An edge that's been seen but not yet believed: handleReceivedEdge() holds it here instead of
    // applying it to rxState immediately, because a genuine noise glitch looks identical to a real
    // edge until the *next* edge shows how long the run it opened actually lasted. It's applied to
    // rxState -- becoming "real" -- once RX_NOISE_MAX_US passes without a glitch-close arriving,
    // either because the next edge is that far out, or because rxNoiseTimer's timeout fires first
    // (needed so the very last edge of a burst, with no further edge ever coming, still lands).
    bool rxPendingValid = false;
    uint32_t rxPendingEdgeTicks = 0; // raw MCPWM capture-timer ticks -- see rxCaptureTicksPerUs
    int rxPendingLevel = 0;
    struct esp_timer* rxNoiseTimer = nullptr;

    // The very last edge handleReceivedEdge() actually received from the MCPWM capture ISR,
    // regardless of what was subsequently done with it -- applied, buffered as rxPending*, or
    // discarded outright as a glitch (which, unlike rxPending*/rxState.lastEdge*, leaves no other
    // trace of the edge anywhere once it's cancelled). Used purely to detect a spurious duplicate
    // redispatch of the same underlying hardware capture event before it can be mistaken for a new
    // one -- see the comment in handleReceivedEdge(). -1 means none received yet this session.
    uint32_t rxLastReceivedTicks = 0;
    int rxLastReceivedLevel = -1;

    // Hardware edge timing for DIO2: the MCPWM capture channel latches (tick count, edge polarity)
    // atomically at the instant of the electrical edge, so onDio2Capture()'s timestamp can't be
    // corrupted by ISR scheduling delay the way a software esp_timer_get_time() read could be --
    // see configureDioPins() for why that matters on this board.
    mcpwm_cap_timer_handle_t rxCaptureTimer = nullptr;
    mcpwm_cap_channel_handle_t rxCaptureChannel = nullptr;
    // ESP32's capture timer clock is hardwired to the APB clock (80MHz) -- the resolution_hz
    // requested in configureDioPins() is only a hint the driver is free to ignore on this target,
    // and does here, so cap_value arrives in raw ~12.5ns ticks, not microseconds. This is the
    // actual resolution read back after creating the timer, divided into every duration *after* a
    // wraparound-safe raw-tick subtraction (see applyEdgeToState/handleReceivedEdge) -- converting
    // each absolute cap_value to microseconds before subtracting would wrap at the wrong modulus
    // (the raw 32-bit register wraps at 2^32 ticks; a pre-divided value would wrap far earlier,
    // corrupting any duration whose two edges straddle that point).
    uint32_t rxCaptureTicksPerUs = 1;

    // Ground-truth edge counter for DIO2, running in parallel with the MCPWM capture channel above:
    // PCNT's counter register increments directly from GPIO transitions in hardware, needing no
    // interrupt/CPU servicing of its own at all (see configureEdgeCounter() -- no watch point is
    // registered, so this unit doesn't even allocate an interrupt), so unlike the single-latch MCPWM
    // capture channel it can't itself silently lose a count under interrupt-dispatch pressure.
    // handleReceivedEdge() polls it on every edge, comparing it against its own per-ISR-call tally
    // (rxIsrEdgeCount), to detect when MCPWM capture coalesced >=2 real edges into one callback.
    pcnt_unit_t* rxEdgeCountUnit = nullptr;
    pcnt_chan_t* rxEdgeCountChannel = nullptr;
    // Raw per-ISR-call edge count (glitches included, unlike rxState.bitCount), periodically reset
    // back to 0 (alongside the PCNT hardware counter) well before either could approach the 16-bit
    // hardware register's real ceiling -- see PCNT_RESET_THRESHOLD. Only ever touched under
    // rxStateMux, and only from handleReceivedEdge (ISR context).
    int32_t rxIsrEdgeCount = 0;

    portMUX_TYPE rxStateMux = portMUX_INITIALIZER_UNLOCKED;
};
