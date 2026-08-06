#pragma once

#include <cstdint>

// sends radio commands received by RadioController
class SX1276Driver
{
public:
    void init();
    void send(uint8_t* data);
};
