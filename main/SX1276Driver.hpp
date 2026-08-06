#pragma once

// sends radio commands received by RadioController
class SX1276Driver
{
public:
    void init();
    void send();
};
