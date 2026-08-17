#include "sdkconfig.h"
#include "CliCommands.hpp"

#if CONFIG_ENABLE_CHIP_SHELL

#include <cstdio>
#include <cstdlib>

#include <argtable3/argtable3.h>
#include <esp_console.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "SX1278Driver.hpp"

namespace
{
constexpr char* TAG = "CliCommands";

// ── sx1278reg: peek/poke a raw SX1278 register, for interactively tuning RF settings without
// reflashing -- primarily RegOokFix (0x15), per the datasheet's own recommended procedure for
// finding the OOK floor threshold that rejects ambient noise on this specific board/antenna
// (see configureReceiver() in SX1278Driver.cpp). Avoid using this while a blind is moving: pokes
// go through the same SPI device senderTask uses, and while individual register transactions are
// serialized safely, interleaving a poke with the middle of a transmission can still corrupt it.
struct RegArgs
{
    arg_str* address;
    arg_str* value;
    // "struct" is required here: argtable3 also declares a function named arg_end(int), which in
    // C++ hides the struct tag of the same name for unqualified lookup.
    struct arg_end* end;
} regArgs;

bool parseByte(const char* text, uint8_t& outValue)
{
    char* end = nullptr;
    unsigned long parsed = std::strtoul(text, &end, 0); // base 0: accepts "0x.." as well as plain decimal
    if (end == text || *end != '\0' || parsed > 0xFF)
    {
        return false;
    }
    outValue = static_cast<uint8_t>(parsed);
    return true;
}

int handleRegCommand(void* context, int argc, char** argv)
{
    auto* driver = static_cast<SX1278Driver*>(context);

    int errors = arg_parse(argc, argv, reinterpret_cast<void**>(&regArgs));
    if (errors != 0)
    {
        arg_print_errors(stderr, regArgs.end, argv[0]);
        return 1;
    }

    uint8_t address = 0;
    if (!parseByte(regArgs.address->sval[0], address))
    {
        printf("Invalid register address '%s'\n", regArgs.address->sval[0]);
        return 1;
    }

    if (regArgs.value->count > 0)
    {
        uint8_t value = 0;
        if (!parseByte(regArgs.value->sval[0], value))
        {
            printf("Invalid register value '%s'\n", regArgs.value->sval[0]);
            return 1;
        }
        if (!driver->pokeRegister(address, value))
        {
            printf("SX1278 not initialised\n");
            return 1;
        }
    }

    uint8_t readBack = 0;
    if (!driver->peekRegister(address, readBack))
    {
        printf("SX1278 not initialised\n");
        return 1;
    }
    printf("reg 0x%02X = 0x%02X\n", address, readBack);
    return 0;
}

// ── sx1278rssi: streams instantaneous RSSI (dBm) while the radio idles in receive mode. Useful
// for empirically separating the ambient noise floor from the remote's actual signal strength --
// watch it while walking the remote away from the receiver, or while raising RegOokFix with
// sx1278reg until it settles instead of chattering with no transmitter active.
struct RssiArgs
{
    arg_int* count;
    arg_int* interval_ms;
    struct arg_end* end; // see the "struct" note on RegArgs::end above
} rssiArgs;

int handleRssiCommand(void* context, int argc, char** argv)
{
    auto* driver = static_cast<SX1278Driver*>(context);

    int errors = arg_parse(argc, argv, reinterpret_cast<void**>(&rssiArgs));
    if (errors != 0)
    {
        arg_print_errors(stderr, rssiArgs.end, argv[0]);
        return 1;
    }

    int count = rssiArgs.count->count > 0 ? rssiArgs.count->ival[0] : 1;
    int intervalMs = rssiArgs.interval_ms->count > 0 ? rssiArgs.interval_ms->ival[0] : 500;
    if (count < 1)
    {
        count = 1;
    }
    if (count > 50) // keep an accidental "sx1278rssi -n 100000" from hanging the console
    {
        count = 50;
    }
    if (intervalMs < 20)
    {
        intervalMs = 20;
    }

    for (int i = 0; i < count; ++i)
    {
        double dbm = 0.0;
        if (!driver->peekRssiDbm(dbm))
        {
            printf("SX1278 not initialised\n");
            return 1;
        }
        printf("RSSI: %.1f dBm\n", dbm);
        if (i + 1 < count)
        {
            vTaskDelay(pdMS_TO_TICKS(intervalMs));
        }
    }
    return 0;
}
} // namespace

// ── public entry point ───────────────────────────────────────────────────────

void cli_register_commands(SX1278Driver& sx1278Driver)
{
    regArgs.address = arg_str1(nullptr, nullptr, "<address>", "register address, e.g. 0x15");
    regArgs.value = arg_str0(nullptr, nullptr, "<value>", "value to write, e.g. 0x20 (omit to just read)");
    regArgs.end = arg_end(2);
    const esp_console_cmd_t regCmd = {
        .command = "sx1278reg",
        .help = "Read (and optionally write) a raw SX1278 register",
        .hint = nullptr,
        .func = nullptr,
        .argtable = &regArgs,
        .func_w_context = &handleRegCommand,
        .context = &sx1278Driver,
    };
    esp_err_t err = esp_console_cmd_register(&regCmd);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register sx1278reg command: %d", err);
    }

    rssiArgs.count = arg_int0("n", nullptr, "<count>", "number of samples to print (default 1, max 50)");
    rssiArgs.interval_ms = arg_int0("d", nullptr, "<ms>", "delay between samples in ms (default 500)");
    rssiArgs.end = arg_end(2);
    const esp_console_cmd_t rssiCmd = {
        .command = "sx1278rssi",
        .help = "Print the SX1278's instantaneous RSSI while idling in receive mode",
        .hint = nullptr,
        .func = nullptr,
        .argtable = &rssiArgs,
        .func_w_context = &handleRssiCommand,
        .context = &sx1278Driver,
    };
    err = esp_console_cmd_register(&rssiCmd);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register sx1278rssi command: %d", err);
    }
}

#endif // CONFIG_ENABLE_CHIP_SHELL
