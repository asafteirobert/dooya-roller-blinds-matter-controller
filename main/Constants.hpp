#pragma once
#include <cstdint>

#include "driver/gpio.h"

/** Factory-reset button (active-LOW, internal pull-up) */
#define BOARD_BUTTON_GPIO GPIO_NUM_0

#define SX1278_NSS_GPIO GPIO_NUM_5
#define SX1278_RESET_GPIO GPIO_NUM_17
#define SX1278_DIO0_GPIO GPIO_NUM_4
#define SX1278_DIO1_GPIO GPIO_NUM_16
// Raw OOK "Data" output (continuous mode), used to receive commands sent by the physical remote.
// Only meaningful while the chip is in receive mode; see SX1278Driver's RX decode state machine.
#define SX1278_DIO2_GPIO GPIO_NUM_27

// Maximum number of blinds this firmware can be configured to control. Shared between app_main
// (which sizes its per-blind endpoint/controller arrays with it) and RadioController (which
// sizes its remote-command routing table with it), so the two stay in sync.
constexpr uint8_t MAX_BLINDS = 8;
