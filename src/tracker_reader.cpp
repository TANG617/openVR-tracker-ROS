#include "tracker/tracker_reader.hpp"
#include "tracker/openvr_context.hpp"
#include <chrono>
#include <sstream>

namespace tracker
{

TrackerReader::TrackerReader(const std::string& serial_number, LogCallback log_callback)
    : serial_number_(serial_number),
      log_callback_(std::move(log_callback))
{
    auto& ctx = OpenVRContext::instance();

    if (!ctx.isInitialized())
    {
        ctx.setLogCallback(log_callback_);
        if (!ctx.initialize())
        {
            log("ERROR", "Failed to initialize OpenVR");
            return;
        }
    }

    auto result = ctx.findTrackerBySerial(serial_number_);
    if (result.has_value())
    {
        device_id_ = result.value();
        device_found_ = true;
        std::ostringstream msg;
        msg << "Found tracker: " << serial_number_ << " (device ID: " << device_id_ << ")";
        log("INFO", msg.str());
    }
    else
    {
        log("ERROR", "Tracker not found: " + serial_number_);
    }
}

void TrackerReader::log(const std::string& level, const std::string& message)
{
    if (log_callback_)
    {
        log_callback_(level, message);
    }
}

bool TrackerReader::getPose(TrackerPose& pose)
{
    auto& ctx = OpenVRContext::instance();
    if (!ctx.isInitialized())
    {
        return false;
    }

    for (int retry = 0; retry < 2; retry++)
    {
        if (!device_found_)
        {
            auto result = ctx.findTrackerBySerial(serial_number_);
            if (!result.has_value())
            {
                return false;
            }
            device_id_ = result.value();
            device_found_ = true;
            consecutive_failures_ = 0;
        }

        double now = currentTimestamp();
        if (now - ctx.lastUpdateTimestamp() > OpenVRContext::kCacheValidityMs)
        {
            if (now - last_cache_warn_time_ > 5000.0)
            {
                std::ostringstream msg;
                msg << "Pose cache is stale (" << static_cast<int>(now - ctx.lastUpdateTimestamp())
                    << "ms old), call updateAllPoses() first";
                log("WARN", msg.str());
                last_cache_warn_time_ = now;
            }
        }

        const vr::TrackedDevicePose_t& device_pose = ctx.getPose(device_id_);

        if (!device_pose.bDeviceIsConnected || !device_pose.bPoseIsValid)
        {
            if (!tryReconnect())
            {
                return false;
            }
            continue;
        }

        consecutive_failures_ = 0;
        first_failure_time_ = -1.0;

        pose.device_id = device_id_;
        pose.timestamp = currentTimestamp();
        pose.is_valid = true;

        const vr::HmdMatrix34_t& openvr_matrix = device_pose.mDeviceToAbsoluteTracking;
        pose.matrix = Eigen::Map<const Eigen::Matrix<float, 3, 4, Eigen::RowMajor>>(
            &openvr_matrix.m[0][0]);

        return true;
    }

    return false;
}

bool TrackerReader::tryReconnect()
{
    auto& ctx = OpenVRContext::instance();
    double now = currentTimestamp();

    consecutive_failures_++;

    if (first_failure_time_ < 0.0)
    {
        first_failure_time_ = now;
    }

    double time_since_last_reconnect = now - last_reconnect_warn_time_;
    if (time_since_last_reconnect >= kReconnectIntervalMs)
    {
        double disconnected_duration = now - first_failure_time_;
        std::ostringstream msg;
        msg << "Tracker " << serial_number_ << " disconnected for "
            << static_cast<int>(disconnected_duration / 1000.0)
            << "s (" << consecutive_failures_ << " failures), attempting reconnect...";
        log("WARN", msg.str());

        device_found_ = false;
        auto result = ctx.findTrackerBySerial(serial_number_);
        if (result.has_value())
        {
            device_id_ = result.value();
            device_found_ = true;
            consecutive_failures_ = 0;
            first_failure_time_ = -1.0;

            std::ostringstream success_msg;
            success_msg << "Tracker " << serial_number_ << " reconnected (device ID: " << device_id_ << ")";
            log("INFO", success_msg.str());
            return true;
        }
        else
        {
            log("WARN", "Tracker " + serial_number_ + " reconnect failed, will retry in 10s");
        }

        last_reconnect_warn_time_ = now;
    }

    return false;
}

bool TrackerReader::isConnected() const
{
    if (!device_found_)
    {
        return false;
    }

    auto& ctx = OpenVRContext::instance();
    if (!ctx.system())
    {
        return false;
    }

    return ctx.system()->IsTrackedDeviceConnected(device_id_);
}

double TrackerReader::currentTimestamp() const
{
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(now.time_since_epoch()).count();
}

}

