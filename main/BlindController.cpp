#include "BlindController.hpp"

#include <utility>

#include <esp_log.h>

BlindController::~BlindController()
{
    if (this->stopTimer != nullptr)
    {
        esp_timer_stop(this->stopTimer);
        esp_timer_delete(this->stopTimer);
    }
    if (this->mutex != nullptr)
    {
        vSemaphoreDelete(this->mutex);
    }
}

void BlindController::init(RadioController &radioController, PositionChangedCallback onPositionChanged,
                            uint8_t initialPercentage)
{
    this->radioController = &radioController;
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
    this->moveToInternal(percentage, true);
}

void BlindController::handleRemoteCommand(RadioController::Command command)
{
    switch (command)
    {
    case RadioController::Command::UP:
        this->moveToInternal(0, false);
        break;
    case RadioController::Command::DOWN:
        this->moveToInternal(100, false);
        break;
    case RadioController::Command::STOP:
        this->stopInternal(false);
        break;
    }
}

void BlindController::moveToInternal(uint8_t percentage, bool sendRadioCommand)
{
    if (percentage > 100)
    {
        percentage = 100;
    }

    xSemaphoreTake(this->mutex, portMAX_DELAY);

    // A duplicate of the move already in flight (e.g. a physical remote repeating the same
    // button while held): treating it as a fresh move would reset moveStartTimeUs, discarding
    // the fractional travel progress made since the move actually started and dragging the
    // position estimate away from where the blind really is. Just ignore it.
    if (this->direction != Direction::NONE && this->targetPercentage == percentage)
    {
        xSemaphoreGive(this->mutex);
        return;
    }

    // A move is already in flight: figure out where the blind actually is before deciding
    // what to do next, and cancel the pending STOP for the old target.
    if (this->direction != Direction::NONE)
    {
        this->cancelStopTimerLocked();
        this->syncPercentageToElapsedTravelLocked();
    }

    this->targetPercentage = percentage;

    if (this->targetPercentage == this->currentPercentage)
    {
        if (this->direction != Direction::NONE)
        {
            ESP_LOGI(TAG, "Blind %d: new target %d matches current position, stopping", this->blindRadioChannel, percentage);
            if (sendRadioCommand)
            {
                this->radioController->sendCommand(this->blindRemoteId, this->blindRadioChannel, RadioController::Command::STOP);
            }
        }
        this->direction = Direction::NONE;
    }
    else
    {
        Direction newDirection = (this->targetPercentage > this->currentPercentage) ? Direction::DOWN : Direction::UP;
        int diff = static_cast<int>(this->targetPercentage) - static_cast<int>(this->currentPercentage);
        if (diff < 0)
        {
            diff = -diff;
        }
        uint32_t travelMiliseconds = (newDirection == Direction::DOWN) ? this->blindLowerTimeMs : this->blindRiseTimeMs;

        ESP_LOGI(TAG, "Blind %d: moving from %d to %d (%s)", this->blindRadioChannel, this->currentPercentage, this->targetPercentage,
                 newDirection == Direction::DOWN ? "DOWN" : "UP");

        this->direction = newDirection;
        this->moveStartPercentage = this->currentPercentage;
        this->moveStartTimeUs = esp_timer_get_time();

        if (sendRadioCommand)
        {
            this->radioController->sendCommand(this->blindRemoteId, this->blindRadioChannel, newDirection == Direction::DOWN ? RadioController::Command::DOWN
                                                                                   : RadioController::Command::UP);
        }

        uint64_t travelTimeUs = static_cast<uint64_t>(diff) * travelMiliseconds * 1000ULL / 100;
        if (travelTimeUs == 0)
        {
            travelTimeUs = 1;
        }
        esp_timer_start_once(this->stopTimer, travelTimeUs);
    }

    uint8_t percentageToReport = this->currentPercentage;
    xSemaphoreGive(this->mutex);

    if (this->onPositionChanged)
    {
        this->onPositionChanged(percentageToReport);
    }
}

void BlindController::stop()
{
    this->stopInternal(true);
}

void BlindController::stopInternal(bool sendRadioCommand)
{
    xSemaphoreTake(this->mutex, portMAX_DELAY);

    bool wasMoving = (this->direction != Direction::NONE);
    if (wasMoving)
    {
        this->cancelStopTimerLocked();
        this->syncPercentageToElapsedTravelLocked();

        ESP_LOGI(TAG, "Blind %d: stop requested at %d%%", this->blindRadioChannel, this->currentPercentage);
        if (sendRadioCommand)
        {
            this->radioController->sendCommand(this->blindRemoteId, this->blindRadioChannel, RadioController::Command::STOP);
        }

        this->direction = Direction::NONE;
        this->targetPercentage = this->currentPercentage;
    }

    uint8_t percentageToReport = this->currentPercentage;
    xSemaphoreGive(this->mutex);

    if (wasMoving && this->onPositionChanged)
    {
        this->onPositionChanged(percentageToReport);
    }
}

uint8_t BlindController::getPositionPercentage() const
{
    xSemaphoreTake(this->mutex, portMAX_DELAY);
    uint8_t percentage = this->currentPercentage;
    xSemaphoreGive(this->mutex);
    return percentage;
}

bool BlindController::isMoving() const
{
    xSemaphoreTake(this->mutex, portMAX_DELAY);
    bool moving = (this->direction != Direction::NONE);
    xSemaphoreGive(this->mutex);
    return moving;
}

// Called with the mutex held. Estimates how far the blind has travelled since the current
// move started and folds that into currentPercentage, without touching the radio or the timer.
void BlindController::syncPercentageToElapsedTravelLocked()
{
    if (this->direction == Direction::NONE)
    {
        return;
    }

    int totalDiff = static_cast<int>(this->targetPercentage) - static_cast<int>(this->moveStartPercentage);
    if (totalDiff < 0)
    {
        totalDiff = -totalDiff;
    }

    uint32_t travelMiliseconds = (this->direction == Direction::DOWN) ? this->blindLowerTimeMs : this->blindRiseTimeMs;
    if (totalDiff == 0 || travelMiliseconds == 0)
    {
        this->currentPercentage = this->targetPercentage;
        return;
    }

    int64_t totalTravelUs = static_cast<int64_t>(totalDiff) * travelMiliseconds * 1000LL / 100;
    int64_t elapsedUs = esp_timer_get_time() - this->moveStartTimeUs;
    if (elapsedUs >= totalTravelUs)
    {
        this->currentPercentage = this->targetPercentage;
        return;
    }

    int percentTravelled = static_cast<int>(elapsedUs * totalDiff / totalTravelUs);
    this->currentPercentage = (this->direction == Direction::DOWN) ? static_cast<uint8_t>(this->moveStartPercentage + percentTravelled)
                                                        : static_cast<uint8_t>(this->moveStartPercentage - percentTravelled);
}

// Called with the mutex held.
void BlindController::cancelStopTimerLocked()
{
    if (esp_timer_is_active(this->stopTimer))
    {
        esp_timer_stop(this->stopTimer);
    }
}

void BlindController::stopTimerCallback(void *arg)
{
    static_cast<BlindController *>(arg)->onTravelComplete();
}

// Runs on the esp_timer service task once the computed travel time has elapsed.
void BlindController::onTravelComplete()
{
    xSemaphoreTake(this->mutex, portMAX_DELAY);

    bool wasMoving = (this->direction != Direction::NONE);
    if (wasMoving)
    {
        this->currentPercentage = this->targetPercentage;
        this->direction = Direction::NONE;

        // At the fully open/closed ends the motor has its own limit switch, so let it run
        // there and self-stop instead of sending STOP. This clears any position estimate
        // error that may have accumulated from prior moves.
        if (this->currentPercentage == 0 || this->currentPercentage == 100)
        {
            ESP_LOGI(TAG, "Blind %d: reached end position %d%%, letting motor self-stop", this->blindRadioChannel, this->currentPercentage);
        }
        else
        {
            ESP_LOGI(TAG, "Blind %d: reached target %d%%, stopping", this->blindRadioChannel, this->currentPercentage);
            this->radioController->sendCommand(this->blindRemoteId, this->blindRadioChannel, RadioController::Command::STOP);
        }
    }

    uint8_t percentageToReport = this->currentPercentage;
    xSemaphoreGive(this->mutex);

    if (wasMoving && this->onPositionChanged)
    {
        this->onPositionChanged(percentageToReport);
    }
}
