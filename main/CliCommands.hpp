#pragma once
#include "sdkconfig.h"

class SX1278Driver;

#if CONFIG_ENABLE_CHIP_SHELL
void cli_register_commands(SX1278Driver& sx1278Driver);
#endif
