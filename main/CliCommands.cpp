#include "sdkconfig.h"
#include "CliCommands.hpp"

#if CONFIG_ENABLE_CHIP_SHELL

#include <esp_console.h>
#include <argtable3/argtable3.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <cstring>

#include "Constants.hpp"

// ── public entry point ───────────────────────────────────────────────────────

void cli_register_commands()
{
}

#endif // CONFIG_ENABLE_CHIP_SHELL