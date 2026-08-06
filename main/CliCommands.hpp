#pragma once
#include "sdkconfig.h"

#if CONFIG_ENABLE_CHIP_SHELL
void cli_register_commands();
#endif