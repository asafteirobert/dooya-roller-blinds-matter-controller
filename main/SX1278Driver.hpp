#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

// Avoids pulling driver/spi_master.h (and its transitive ESP-IDF includes) into every file that
// includes this header; spi_device_handle_t is just a pointer to this struct.
struct spi_device_t;
// Same trick for esp_timer.h; esp_timer_handle_t is just a pointer to this struct.
struct esp_timer;

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
//   in that mode; a GPIO interrupt timestamps every edge and decodes it in software (mirroring
//   the encoding transmitWaveform() produces), handing off completed frames to a receiver task
//   which invokes the callback registered via setReceiveCallback().
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
        };
        Kind kind = Kind::None;
        int64_t durationUs = 0;
        uint8_t bitCount = 0; // bits collected so far, for GlitchDropped/AbortedFrame/BitInvalid
    };

    void resetChip();
    void configureDioPins();
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
    static void dio2IsrHandler(void* arg);
    void handleReceivedEdge(int64_t nowUs, int level);
    bool applyEdgeToState(int64_t edgeUs, int level, std::array<uint8_t, 5>& outCompletedFrame, RxEvent& outEvent);
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
        int64_t lastEdgeUs = 0;
    } rxState;

    // An edge that's been seen but not yet believed: handleReceivedEdge() holds it here instead of
    // applying it to rxState immediately, because a genuine noise glitch looks identical to a real
    // edge until the *next* edge shows how long the run it opened actually lasted. It's applied to
    // rxState -- becoming "real" -- once RX_NOISE_MAX_US passes without a glitch-close arriving,
    // either because the next edge is that far out, or because rxNoiseTimer's timeout fires first
    // (needed so the very last edge of a burst, with no further edge ever coming, still lands).
    bool rxPendingValid = false;
    int64_t rxPendingEdgeUs = 0;
    int rxPendingLevel = 0;
    struct esp_timer* rxNoiseTimer = nullptr;

    portMUX_TYPE rxStateMux = portMUX_INITIALIZER_UNLOCKED;
};
