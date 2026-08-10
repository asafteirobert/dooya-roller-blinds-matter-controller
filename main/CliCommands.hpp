#pragma once
#include "sdkconfig.h"

class SX1276Driver;

#if CONFIG_ENABLE_CHIP_SHELL
void cli_register_commands(SX1276Driver& sx1276Driver);
#endif
