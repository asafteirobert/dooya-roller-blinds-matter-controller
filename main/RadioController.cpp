#include "RadioController.hpp"
#include "BlindController.hpp"

#include <array>
#include <esp_log.h>

void RadioController::init(SX1278Driver& sx1278Driver)
{
    this->driver = &sx1278Driver;
    this->driver->setReceiveCallback([this](const std::array<uint8_t, 5>& data) { this->onFrameReceived(data); });
}

void RadioController::sendCommand(uint32_t remoteId, uint8_t channel, Command command)
{
    ESP_LOGI(TAG, "Received command %d for channel %d", static_cast<int>(command), channel);

    uint8_t button = static_cast<uint8_t>(command) & 0x0F;
    //uint8_t check = (~button) & 0x0F;
    uint8_t check = button;

    std::array<uint8_t, 5> data = {
        static_cast<uint8_t>((remoteId >> 16) & 0xFF),
        static_cast<uint8_t>((remoteId >> 8) & 0xFF),
        static_cast<uint8_t>(remoteId & 0xFF),
        channel,
        static_cast<uint8_t>((button << 4) | check),
    };

    this->driver->send(data, 3);
}

void RadioController::registerBlind(BlindController& blindController)
{
    if (this->registeredBlindCount >= this->registeredBlinds.size())
    {
        ESP_LOGE(TAG, "Too many blinds registered, remote commands for this blind will be ignored");
        return;
    }
    this->registeredBlinds[this->registeredBlindCount++] = &blindController;
}

// Runs on SX1278Driver's receiver task (never from ISR context), once per frame decoded off the
// air while the radio is idling in receive mode.
void RadioController::onFrameReceived(const std::array<uint8_t, 5>& data)
{
    uint32_t remoteId = (static_cast<uint32_t>(data[0]) << 16) | (static_cast<uint32_t>(data[1]) << 8) | data[2];
    uint8_t channel = data[3];
    uint8_t button = (data[4] >> 4) & 0x0F;

    Command command = Command::STOP;
    switch (button)
    {
    case static_cast<uint8_t>(Command::UP):
        command = Command::UP;
        break;
    case static_cast<uint8_t>(Command::DOWN):
        command = Command::DOWN;
        break;
    case static_cast<uint8_t>(Command::STOP):
        command = Command::STOP;
        break;
    default:
        // Real Dooya remotes have other buttons (pairing/program, etc.) that legitimately show
        // up here; this isn't an error condition.
        ESP_LOGD(TAG, "Ignoring remote frame with unhandled button code %d", button);
        return;
    }

    ESP_LOGI(TAG, "Received remote command %d from remote 0x%06lX channel %d", static_cast<int>(command),
             static_cast<unsigned long>(remoteId), channel);

    for (uint8_t i = 0; i < this->registeredBlindCount; ++i)
    {
        BlindController* blind = this->registeredBlinds[i];
        if (blind->blindRemoteId == remoteId && blind->blindRadioChannel == channel)
        {
            blind->handleRemoteCommand(command);
            return;
        }
    }

    ESP_LOGD(TAG, "No blind registered for remote 0x%06lX channel %d", static_cast<unsigned long>(remoteId), channel);
}
