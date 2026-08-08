#include "BlindController.hpp"

#include <utility>

#include <esp_log.h>

BlindController::~BlindController()
{
    if (stopTimer != nullptr)
    {
        esp_timer_stop(stopTimer);
        esp_timer_delete(stopTimer);
    }
    if (mutex != nullptr)
    {
        vSemaphoreDelete(mutex);
    }
}

void BlindController::init(RadioController &radioController, uint8_t blindId, PositionChangedCallback onPositionChanged,
                            uint8_t initialPercentage)
{
    this->radioController = &radioController;
    this->blindId = blindId;
    this->onPositionChanged = std::move(onPositionChanged);
    this->mutex = xSemaphoreCreateMutex();

    // Restore the position estimate persisted from before a reset, instead of assuming fully open.
    this->currentPercentage = initialPercentage;
    this->targetPercentage = initialPercentage;

    const esp_timer_create_args_t timerArgs = {
        .callback = &BlindController::stopTimerCallback,
        .arg = this,
        .name = "blind_stop",
    };
    esp_timer_create(&timerArgs, &this->stopTimer);
}

void BlindController::moveTo(uint8_t percentage)
{
    if (percentage > 100)
    {
        percentage = 100;
    }

    xSemaphoreTake(mutex, portMAX_DELAY);

    // A move is already in flight: figure out where the blind actually is before deciding
    // what to do next, and cancel the pending STOP for the old target.
    if (direction != Direction::NONE)
    {
        cancelStopTimerLocked();
        syncPercentageToElapsedTravelLocked();
    }

    targetPercentage = percentage;

    if (targetPercentage == currentPercentage)
    {
        if (direction != Direction::NONE)
        {
            ESP_LOGI(TAG, "Blind %d: new target %d matches current position, stopping", blindId, percentage);
            radioController->sendCommand(blindId, RadioController::Command::STOP);
        }
        direction = Direction::NONE;
    }
    else
    {
        Direction newDirection = (targetPercentage > currentPercentage) ? Direction::DOWN : Direction::UP;
        int diff = static_cast<int>(targetPercentage) - static_cast<int>(currentPercentage);
        if (diff < 0)
        {
            diff = -diff;
        }
        uint8_t travelSeconds = (newDirection == Direction::DOWN) ? BLIND_LOWER_TIME_SECONDS : BLIND_RISE_TIME_SECONDS;

        ESP_LOGI(TAG, "Blind %d: moving from %d to %d (%s)", blindId, currentPercentage, targetPercentage,
                 newDirection == Direction::DOWN ? "DOWN" : "UP");

        direction = newDirection;
        moveStartPercentage = currentPercentage;
        moveStartTimeUs = esp_timer_get_time();

        radioController->sendCommand(blindId, newDirection == Direction::DOWN ? RadioController::Command::DOWN
                                                                               : RadioController::Command::UP);

        uint64_t travelTimeUs = static_cast<uint64_t>(diff) * travelSeconds * 1000000ULL / 100;
        if (travelTimeUs == 0)
        {
            travelTimeUs = 1;
        }
        esp_timer_start_once(stopTimer, travelTimeUs);
    }

    uint8_t percentageToReport = currentPercentage;
    xSemaphoreGive(mutex);

    if (onPositionChanged)
    {
        onPositionChanged(percentageToReport);
    }
}

void BlindController::stop()
{
    xSemaphoreTake(mutex, portMAX_DELAY);

    bool wasMoving = (direction != Direction::NONE);
    if (wasMoving)
    {
        cancelStopTimerLocked();
        syncPercentageToElapsedTravelLocked();

        ESP_LOGI(TAG, "Blind %d: stop requested at %d%%", blindId, currentPercentage);
        radioController->sendCommand(blindId, RadioController::Command::STOP);

        direction = Direction::NONE;
        targetPercentage = currentPercentage;
    }

    uint8_t percentageToReport = currentPercentage;
    xSemaphoreGive(mutex);

    if (wasMoving && onPositionChanged)
    {
        onPositionChanged(percentageToReport);
    }
}

uint8_t BlindController::getPositionPercentage() const
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    uint8_t percentage = currentPercentage;
    xSemaphoreGive(mutex);
    return percentage;
}

bool BlindController::isMoving() const
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    bool moving = (direction != Direction::NONE);
    xSemaphoreGive(mutex);
    return moving;
}

// Called with the mutex held. Estimates how far the blind has travelled since the current
// move started and folds that into currentPercentage, without touching the radio or the timer.
void BlindController::syncPercentageToElapsedTravelLocked()
{
    if (direction == Direction::NONE)
    {
        return;
    }

    int totalDiff = static_cast<int>(targetPercentage) - static_cast<int>(moveStartPercentage);
    if (totalDiff < 0)
    {
        totalDiff = -totalDiff;
    }

    uint8_t travelSeconds = (direction == Direction::DOWN) ? BLIND_LOWER_TIME_SECONDS : BLIND_RISE_TIME_SECONDS;
    if (totalDiff == 0 || travelSeconds == 0)
    {
        currentPercentage = targetPercentage;
        return;
    }

    int64_t totalTravelUs = static_cast<int64_t>(totalDiff) * travelSeconds * 1000000LL / 100;
    int64_t elapsedUs = esp_timer_get_time() - moveStartTimeUs;
    if (elapsedUs >= totalTravelUs)
    {
        currentPercentage = targetPercentage;
        return;
    }

    int percentTravelled = static_cast<int>(elapsedUs * totalDiff / totalTravelUs);
    currentPercentage = (direction == Direction::DOWN) ? static_cast<uint8_t>(moveStartPercentage + percentTravelled)
                                                        : static_cast<uint8_t>(moveStartPercentage - percentTravelled);
}

// Called with the mutex held.
void BlindController::cancelStopTimerLocked()
{
    if (esp_timer_is_active(stopTimer))
    {
        esp_timer_stop(stopTimer);
    }
}

void BlindController::stopTimerCallback(void *arg)
{
    static_cast<BlindController *>(arg)->onTravelComplete();
}

// Runs on the esp_timer service task once the computed travel time has elapsed.
void BlindController::onTravelComplete()
{
    xSemaphoreTake(mutex, portMAX_DELAY);

    bool wasMoving = (direction != Direction::NONE);
    if (wasMoving)
    {
        currentPercentage = targetPercentage;
        direction = Direction::NONE;

        ESP_LOGI(TAG, "Blind %d: reached target %d%%, stopping", blindId, currentPercentage);
        radioController->sendCommand(blindId, RadioController::Command::STOP);
    }

    uint8_t percentageToReport = currentPercentage;
    xSemaphoreGive(mutex);

    if (wasMoving && onPositionChanged)
    {
        onPositionChanged(percentageToReport);
    }
}
