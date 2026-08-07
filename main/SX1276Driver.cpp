#include "SX1276Driver.hpp"
#include <esp_log.h>

void SX1276Driver::init()
{
}

void SX1276Driver::send(const std::array<uint8_t, 5>& data, uint8_t repeats)
{
    ESP_LOGI(TAG, "Sending %02X %02X %02X %02X %02X, repeats=%d",
             data[0], data[1], data[2], data[3], data[4], repeats);
}
