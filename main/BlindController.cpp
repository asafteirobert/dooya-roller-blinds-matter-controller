#include "BlindController.hpp"

void BlindController::init(RadioController &radioController, uint8_t blindId)
{
    this->radioController = &radioController;
    this->blindId = blindId;
}

void BlindController::moveTo(uint8_t percentage)
{
    //TODO
    this->currentPercentage = percentage;
}

void BlindController::stop()
{
    //TODO
}

uint8_t BlindController::getPositionPercentage() const
{
    //TODO    
    return this->currentPercentage;
}
