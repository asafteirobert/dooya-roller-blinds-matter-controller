#include "BlindController.hpp"

void BlindController::init(RadioController &radioController, uint8_t blindId)
{
    this->radioController = &radioController;
    this->blindId = blindId;
}