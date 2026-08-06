#pragma once
#include "driver/gpio.h"

/** Factory-reset button (active-LOW, internal pull-up) */
#define BOARD_BUTTON_GPIO GPIO_NUM_0

#define SX1276_NSS_GPIO GPIO_NUM_5
#define SX1276_RESET_GPIO GPIO_NUM_17
#define SX1276_DIO0_GPIO GPIO_NUM_4
#define SX1276_DIO1_GPIO GPIO_NUM_16
