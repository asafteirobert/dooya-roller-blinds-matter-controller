#pragma once

#include <cstdint>

#include "SX1276Driver.hpp"
// gets commands from BlindController and sends Dooya radio commands to SX1276Driver
class RadioController
{
    static constexpr char *TAG = "RadioController";
public:
    enum class Command
    {
        UP = 1,
        DOWN = 3,
        STOP = 5
    };

    void init(SX1276Driver& sx1276Driver);
    void sendCommand(uint32_t remoteId, uint8_t channel, Command command);
private:
    SX1276Driver* driver = nullptr;
};