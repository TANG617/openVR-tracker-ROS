#pragma once

#include <string>
#include "tracker/types.hpp"
#include "openvr.h"

namespace tracker
{

class TrackerReader
{
public:
    explicit TrackerReader(const std::string& serial_number, LogCallback log_callback = nullptr);
    ~TrackerReader() = default;

    TrackerReader(const TrackerReader&) = delete;
    TrackerReader& operator=(const TrackerReader&) = delete;
    TrackerReader(TrackerReader&&) = default;
    TrackerReader& operator=(TrackerReader&&) = default;

    [[nodiscard]] bool getPose(TrackerPose& pose);
    [[nodiscard]] bool isConnected() const;

    [[nodiscard]] const std::string& serialNumber() const { return serial_number_; }
    [[nodiscard]] int deviceId() const { return static_cast<int>(device_id_); }

private:
    [[nodiscard]] bool tryReconnect();
    [[nodiscard]] double currentTimestamp() const;
    void log(const std::string& level, const std::string& message);

    std::string serial_number_;
    vr::TrackedDeviceIndex_t device_id_{vr::k_unTrackedDeviceIndexInvalid};
    bool device_found_{false};
    int consecutive_failures_{0};

    LogCallback log_callback_;
    double last_reconnect_warn_time_{0.0};
    double last_cache_warn_time_{0.0};
    double first_failure_time_{-1.0};

    static constexpr double kReconnectIntervalMs = 10000.0;
};

}

