#pragma once

#include <cstdint>

#include "SX1276Driver.hpp"
// gets commands from BlindController and sends Dooya radio commands to SX1276Driver
class RadioController
{
public:
    enum class Command
    {
        UP = 1,
        DOWN = 2,
        STOP = 3
    };

    void init(SX1276Driver& sx1276Driver);
    void sendCommand(uint8_t blind, Command command);
private:
    SX1276Driver* driver = nullptr;
};