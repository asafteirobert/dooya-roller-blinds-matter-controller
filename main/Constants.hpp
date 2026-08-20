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

// Debug-only outputs for probing RX edge timing on a logic analyzer against DIO2: each mirrors the
// level of the edge it's currently processing, so RX_DEBUG_APPLY lagging/missing edges that
// RX_DEBUG_EDGE saw shows exactly which ones were buffered out as noise glitches.
#define SX1278_RX_DEBUG_EDGE_GPIO GPIO_NUM_25  // set in handleReceivedEdge, every raw DIO2 edge
#define SX1278_RX_DEBUG_APPLY_GPIO GPIO_NUM_26 // set in applyEdgeToState, only confirmed edges

// Maximum number of blinds this firmware can be configured to control. Shared between app_main
// (which sizes its per-blind endpoint/controller arrays with it) and RadioController (which
// sizes its remote-command routing table with it), so the two stay in sync.
constexpr uint8_t MAX_BLINDS = 8;
