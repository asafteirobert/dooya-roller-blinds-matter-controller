#pragma once

// Controls one roller blind.
// Receives percentage commands and, based on timing, sends the correct button presses to RadioController.
class BlindController
{
public:
    void init(uint8_t blindId);
    void moveTo(uint8_t percentage);
private:
    uint8_t blindId;
};

