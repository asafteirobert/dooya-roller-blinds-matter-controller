#pragma once

#include <cstdint>
#include "RadioController.hpp"

// Controls one roller blind.
// Receives percentage commands and, based on timing, sends the correct UP/DOWN/STOP button presses to RadioController.
class BlindController
{
public:
    void init(RadioController& radioController, uint8_t blindId);
    void moveTo(uint8_t percentage);
    void stop();
    uint8_t getPositionPercentage() const;
private:
    uint8_t blindId;
    uint8_t currentPercentage = 0;
    RadioController* radioController = nullptr;
};

