#pragma once

#include <optional>
#include <string>
#include "openvr.h"
#include "tracker/types.hpp"

namespace tracker
{

class OpenVRContext
{
public:
    static OpenVRContext& instance();

    OpenVRContext(const OpenVRContext&) = delete;
    OpenVRContext& operator=(const OpenVRContext&) = delete;

    void setLogCallback(LogCallback callback);

    [[nodiscard]] bool initialize();
    void shutdown();
    [[nodiscard]] bool isInitialized() const { return initialized_; }

    void updateAllPoses();
    [[nodiscard]] double lastUpdateTimestamp() const { return last_update_timestamp_; }

    [[nodiscard]] const vr::TrackedDevicePose_t& getPose(vr::TrackedDeviceIndex_t idx) const;
    [[nodiscard]] vr::IVRSystem* system() const { return vr_system_; }

    [[nodiscard]] std::optional<vr::TrackedDeviceIndex_t> findTrackerBySerial(
        const std::string& serial_number);

    static constexpr double kCacheValidityMs = 100.0;

private:
    OpenVRContext() = default;
    ~OpenVRContext();

    void log(const std::string& level, const std::string& message);

    vr::IVRSystem* vr_system_{nullptr};
    bool initialized_{false};
    vr::TrackedDevicePose_t cached_poses_[vr::k_unMaxTrackedDeviceCount]{};
    double last_update_timestamp_{0.0};
    LogCallback log_callback_;
};

}

