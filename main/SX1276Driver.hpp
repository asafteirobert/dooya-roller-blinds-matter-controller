#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

// Avoids pulling driver/spi_master.h (and its transitive ESP-IDF includes) into every file that
// includes this header; spi_device_handle_t is just a pointer to this struct.
struct spi_device_t;

// Drives an SX1276 module over VSPI to replay Dooya remote control commands.
//
// The chip has no built-in Dooya support, so the 40-bit command is hand-encoded into the exact
// OOK/PWM pulse train and streamed through the SX1276's TX FIFO as one raw fixed-length packet
// (preamble/sync/CRC all disabled).
//
// send() only enqueues the command and returns immediately; a dedicated background task owns the
// SPI device and transmits queued commands one at a time, in the order they were submitted, so
// callers (Matter/timer callbacks) never block on the air time of a transmission.
class SX1276Driver
{
    static constexpr char *TAG = "SX1276Driver";
public:
    void init();
    void send(const std::array<uint8_t, 5>& data, uint8_t repeats = 1);

private:
    struct QueuedCommand
    {
        std::array<uint8_t, 5> data;
        uint8_t repeats;
    };

    void resetChip();
    void configureDioPins();
    void configureRadio();
    void transmitWaveform(const std::vector<uint8_t>& waveform);
    void sendNow(const std::array<uint8_t, 5>& data, uint8_t repeats);

    void writeRegister(uint8_t address, uint8_t value);
    uint8_t readRegister(uint8_t address);
    void writeFifo(const uint8_t* data, size_t length);

    static void senderTask(void* arg);

    spi_device_t* spiDevice = nullptr;
    QueueHandle_t commandQueue = nullptr;
    TaskHandle_t senderTaskHandle = nullptr;
};
