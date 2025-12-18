#include "tracker/steamvr_manager.hpp"
#include <array>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <cstdio>

namespace tracker
{

SteamVRManager::SteamVRManager(LogCallback log_callback)
    : log_callback_(std::move(log_callback))
{
}

void SteamVRManager::log(const std::string& level, const std::string& message)
{
    if (log_callback_)
    {
        log_callback_(level, message);
    }
}

bool SteamVRManager::start()
{
    if (isSteamVRRunning())
    {
        log("INFO", "SteamVR already running, skipping launch");
        return true;
    }

    return launchSteamVR();
}

bool SteamVRManager::isSteamVRRunning()
{
    FILE* fp = popen("pgrep vrserver 2>/dev/null", "r");
    if (fp)
    {
        std::array<char, 16> buf{};
        bool running = (fgets(buf.data(), buf.size(), fp) != nullptr);
        pclose(fp);
        return running;
    }
    return false;
}

bool SteamVRManager::launchSteamVR()
{
    const char* display = getenv("DISPLAY");
    if (!display)
        display = ":0";

    const char* home = getenv("HOME");
    if (!home)
        home = "/root";

    std::array<char, 1024> cmd{};
    snprintf(cmd.data(), cmd.size(),
             "env -i bash -c \""
             "export PATH='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'; "
             "export HOME='%s'; export DISPLAY='%s'; "
             "cd /opt/steamvr && ./start_steamvr_offline.sh\" "
             "> %s/steamvr_launch.log 2>&1 &",
             home, display, home);

    log("INFO", "Starting SteamVR...");
    if (system(cmd.data()) != 0)
    {
        log("ERROR", "Failed to execute SteamVR launch command");
        return false;
    }

    log("INFO", "Waiting 5 seconds for SteamVR initialization...");
    std::this_thread::sleep_for(std::chrono::seconds(5));
    log("INFO", "SteamVR started successfully");
    return true;
}

}
