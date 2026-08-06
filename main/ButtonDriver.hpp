#pragma once
#include <button_gpio.h>
#include <iot_button.h>

class ButtonDriver
{
    static constexpr char *TAG = "app_button_driver";
    public:
        void init();
        button_handle_t getButtonHandle();
    private:
        static void buttonClickCallback(void* handle, void* userData);
        static void buttonLongPressHoldCallback(void *handle, void *userData);
        static void buttonPressUpCallback(void *handle, void *userData);
        button_handle_t handle = nullptr;
        bool performFactoryReset = false;
};