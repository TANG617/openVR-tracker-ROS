#pragma once

#include "tracker/types.hpp"

namespace tracker
{

class SteamVRManager
{
public:
    explicit SteamVRManager(LogCallback log_callback = nullptr);
    ~SteamVRManager() = default;
    SteamVRManager(const SteamVRManager&) = delete;
    SteamVRManager& operator=(const SteamVRManager&) = delete;

    [[nodiscard]] bool start();

private:
    [[nodiscard]] bool isSteamVRRunning();
    [[nodiscard]] bool launchSteamVR();
    void log(const std::string& level, const std::string& message);

    LogCallback log_callback_;
};

}

