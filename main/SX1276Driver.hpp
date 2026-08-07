#pragma once

#include <array>
#include <cstdint>

// sends radio commands received by RadioController
class SX1276Driver
{
    static constexpr char *TAG = "SX1276Driver";
public:
    void init();
    void send(const std::array<uint8_t, 5>& data, uint8_t repeats = 1);
};
