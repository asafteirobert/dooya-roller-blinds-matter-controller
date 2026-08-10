#pragma once

#include <array>
#include <cstdint>

#include "Constants.hpp"
#include "SX1276Driver.hpp"

class BlindController;

// Bridges BlindController and the physical remote to SX1276Driver's raw radio frames.
//   - Outgoing: gets UP/DOWN/STOP commands from BlindController and sends the corresponding
//     Dooya radio commands to SX1276Driver.
//   - Incoming: decodes frames SX1276Driver receives from the physical remote and routes them to
//     whichever registered BlindController owns that (remoteId, channel) pair, so its position
//     estimate stays in sync even when the remote is used directly.
class RadioController
{
    static constexpr char *TAG = "RadioController";
public:
    enum class Command
    {
        UP = 1,
        DOWN = 3,
        STOP = 5
    };

    void init(SX1276Driver& sx1276Driver);
    void sendCommand(uint32_t remoteId, uint8_t channel, Command command);

    // Makes incoming remote commands addressed to blindController's (remoteId, channel) reach
    // it. Looked up by dereferencing blindController->blindRemoteId/blindRadioChannel at the
    // time a frame arrives (not a snapshot taken here), so it stays correct even after those are
    // changed later, e.g. via a Matter attribute write.
    void registerBlind(BlindController& blindController);

private:
    void onFrameReceived(const std::array<uint8_t, 5>& data);

    SX1276Driver* driver = nullptr;
    std::array<BlindController*, MAX_BLINDS> registeredBlinds{};
    uint8_t registeredBlindCount = 0;
};
