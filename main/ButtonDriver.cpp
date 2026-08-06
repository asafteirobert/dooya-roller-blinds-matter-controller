#include "ButtonDriver.hpp"
#include "Constants.hpp"

#include <esp_err.h>
#include <esp_log.h>
#include <esp_matter.h>

void ButtonDriver::init()
{
    const button_config_t btn_cfg = 
    {
        .long_press_time  = 5000,   // 5 s - factory reset
        .short_press_time = 50,
    };
    const button_gpio_config_t btn_gpio_cfg = 
    {
        .gpio_num = BOARD_BUTTON_GPIO,
        .active_level = 0,
    };

    if (iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &this->handle) != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to create button device");
        return;
    }

    esp_err_t err = ESP_OK;
    err |= iot_button_register_cb(handle, BUTTON_SINGLE_CLICK, NULL, buttonClickCallback, this);
    err |= iot_button_register_cb(handle, BUTTON_LONG_PRESS_HOLD, NULL, buttonLongPressHoldCallback, this);
    err |= iot_button_register_cb(handle, BUTTON_PRESS_UP, NULL, buttonPressUpCallback, this);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set button callbacks");
        return;
    }

    ESP_LOGI(TAG, "Button initialised on GPIO %d", BOARD_BUTTON_GPIO);
    return;
}

void ButtonDriver::buttonClickCallback(void *handle, void *userData)
{
}

void ButtonDriver::buttonLongPressHoldCallback(void *handle, void *userData)
{
    ButtonDriver* thisInstance = (ButtonDriver*)userData;
    if (!thisInstance->performFactoryReset)
    {
        ESP_LOGI(TAG, "Factory reset triggered. Release the button to start factory reset.");
        thisInstance->performFactoryReset = true;
    }
}

void ButtonDriver::buttonPressUpCallback(void *handle, void *userData)
{
    ButtonDriver* thisInstance = (ButtonDriver*)userData;
    if (thisInstance->performFactoryReset)
    {
        ESP_LOGI(TAG, "Starting factory reset");
        esp_matter::factory_reset();
        thisInstance->performFactoryReset = false;
    }
}