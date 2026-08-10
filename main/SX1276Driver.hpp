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

// Drives an SX1276 module over VSPI to replay Dooya remote control commands, and to decode the
// same commands when the physical remote transmits them directly to the blind.
//
// The chip has no built-in Dooya support, so in both directions the 40-bit command is hand-coded
// against the exact OOK/PWM pulse train the remote and motor speak, rather than relying on the
// SX1276's packet engine (which only recognises framing this protocol doesn't have: no
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
class SX1276Driver
{
    static constexpr char *TAG = "SX1276Driver";
public:
    // Invoked, from a dedicated background task (never from ISR context), with every complete
    // 40-bit frame decoded off the air while the radio is idling in receive mode.
    using ReceiveCallback = std::function<void(const std::array<uint8_t, 5>& data)>;

    void init();
    void send(const std::array<uint8_t, 5>& data, uint8_t repeats = 1);
    void setReceiveCallback(ReceiveCallback callback);

    // Diagnostic register access, exposed for interactive RF tuning via the "sx1276reg"/
    // "sx1276rssi" console commands (see CliCommands.cpp). The OOK receive threshold in
    // particular (RegOokFix, see configureReceiver()) is board/environment-specific and can't be
    // gotten right from firmware defaults alone -- the datasheet's own tuning procedure is to
    // adjust it live and watch the result. Safe to call from any task, including concurrently
    // with normal operation: the underlying SPI transaction is serialized against senderTask's
    // own via the SPI driver's bus lock, though poking registers mid-transmission can still
    // disrupt that transmission. Returns false (without touching the bus) if the driver never
    // finished initialising.
    bool peekRegister(uint8_t address, uint8_t& outValue);
    bool pokeRegister(uint8_t address, uint8_t value);
    // Convenience for "sx1276rssi": instantaneous received signal strength while idling in
    // receive mode, in dBm.
    bool peekRssiDbm(double& outDbm);

private:
    struct QueuedCommand
    {
        std::array<uint8_t, 5> data;
        uint8_t repeats;
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

    spi_device_t* spiDevice = nullptr;
    QueueHandle_t commandQueue = nullptr;
    TaskHandle_t senderTaskHandle = nullptr;

    ReceiveCallback receiveCallback;
    QueueHandle_t rxFrameQueue = nullptr;
    TaskHandle_t receiverTaskHandle = nullptr;

    // In-progress RX decode. Touched only from handleReceivedEdge (ISR context, DIO2 interrupt
    // disabled while transmitting) and reset from enterReceiveMode() (task context, only while
    // that interrupt is disabled); rxStateMux makes that handover safe across cores.
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
    portMUX_TYPE rxStateMux = portMUX_INITIALIZER_UNLOCKED;
};
