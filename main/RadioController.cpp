#include "RadioController.hpp"
#include <esp_log.h>

void RadioController::init(SX1276Driver& sx1276Driver)
{
    this->driver = &sx1276Driver;
}

void RadioController::sendCommand(uint8_t blind, Command command)
{
    ESP_LOGI(TAG, "Received command %d for blind %d", command, blind);
}
