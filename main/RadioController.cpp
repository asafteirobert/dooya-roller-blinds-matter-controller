#include "RadioController.hpp"

void RadioController::init(SX1276Driver& sx1276Driver)
{
    this->driver = &sx1276Driver;
}