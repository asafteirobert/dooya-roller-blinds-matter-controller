#pragma once

// gets commands from BlindController and sends Dooya radio commands to SX1276Driver
class RadioController
{
    enum class Command
    {
        UP = 1,
        DOWN = 2,
        STOP = 3
    };

    void init();
    void sendCommand(uint8_t blind, Command command);
};