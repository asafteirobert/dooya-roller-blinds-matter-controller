#include "RadioController.hpp"
#include <array>
#include <esp_log.h>

void RadioController::init(SX1276Driver& sx1276Driver)
{
    this->driver = &sx1276Driver;
}

void RadioController::sendCommand(uint8_t channel, Command command)
{
    ESP_LOGI(TAG, "Received command %d for channel %d", static_cast<int>(command), channel);

    uint8_t button = static_cast<uint8_t>(command) & 0x0F;
    uint8_t check = (~button) & 0x0F;

    std::array<uint8_t, 5> data = {
        static_cast<uint8_t>((REMOTE_ID >> 16) & 0xFF),
        static_cast<uint8_t>((REMOTE_ID >> 8) & 0xFF),
        static_cast<uint8_t>(REMOTE_ID & 0xFF),
        channel,
        static_cast<uint8_t>((button << 4) | check),
    };

    driver->send(data, 3);
}
