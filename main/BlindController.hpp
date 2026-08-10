#pragma once

#include <cstdint>
#include <functional>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "RadioController.hpp"

// Controls one roller blind.
// Receives percentage commands and, based on timing, sends the correct UP/DOWN/STOP button presses to RadioController.
// Percentage follows the Matter WindowCovering convention: 0 = fully open, 100 = fully closed.
class BlindController
{
    static constexpr char *TAG = "BlindController";
public:
    // Invoked (from the caller's thread, outside of any internal lock) whenever the current
    // position estimate changes, so the caller can mirror it into the Matter data model.
    using PositionChangedCallback = std::function<void(uint8_t percentage)>;

    ~BlindController();

    void init(RadioController& radioController, PositionChangedCallback onPositionChanged = nullptr,
              uint8_t initialPercentage = 0);
    void moveTo(uint8_t targetPercentage);
    void stop();
    uint8_t getPositionPercentage() const;
    bool isMoving() const;

    // Time it takes to go from fully open to fully closed
    uint32_t blindLowerTimeMs = 24000;
    // Time it takes to go from fully closed to fully open
    uint32_t blindRiseTimeMs = 25000;
    // Remote channel number this instance controls
    uint8_t blindRadioChannel = 1;
    // Dooya remote ID transmitted with each command, as configured on the physical remote
    // this blind is paired to
    uint32_t blindRemoteId = 0x9a66ab;
private:
    enum class Direction
    {
        NONE,
        UP,
        DOWN
    };

    // All of these assume the mutex is already held by the caller.
    void syncPercentageToElapsedTravelLocked();
    void cancelStopTimerLocked();

    static void stopTimerCallback(void *arg);
    void onTravelComplete();

    RadioController *radioController = nullptr;
    PositionChangedCallback onPositionChanged;
    esp_timer_handle_t stopTimer = nullptr;
    mutable SemaphoreHandle_t mutex = nullptr;

    uint8_t currentPercentage = 0;
    uint8_t targetPercentage = 0;
    uint8_t moveStartPercentage = 0;
    Direction direction = Direction::NONE;
    int64_t moveStartTimeUs = 0;
};
