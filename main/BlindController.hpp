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
private:
    uint8_t blindId;
    RadioController* radioController = nullptr;
};

