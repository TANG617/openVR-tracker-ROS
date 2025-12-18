#include "tracker/openvr_context.hpp"
#include <chrono>
#include <sstream>
#include <array>

namespace tracker
{

OpenVRContext& OpenVRContext::instance()
{
    static OpenVRContext ctx;
    return ctx;
}

OpenVRContext::~OpenVRContext()
{
    shutdown();
}

void OpenVRContext::setLogCallback(LogCallback callback)
{
    log_callback_ = std::move(callback);
}

void OpenVRContext::log(const std::string& level, const std::string& message)
{
    if (log_callback_)
    {
        log_callback_(level, message);
    }
}

bool OpenVRContext::initialize()
{
    if (initialized_)
    {
        return true;
    }

    vr::EVRInitError error = vr::VRInitError_None;
    vr_system_ = vr::VR_Init(&error, vr::VRApplication_Background);

    if (error != vr::VRInitError_None && error != vr::VRInitError_Init_AlreadyRunning)
    {
        std::ostringstream msg;
        msg << "Failed to initialize OpenVR: " << vr::VR_GetVRInitErrorAsEnglishDescription(error);
        log("ERROR", msg.str());
        vr_system_ = nullptr;
        return false;
    }

    if (!vr_system_)
    {
        std::ostringstream msg;
        msg << "OpenVR system is null (error code: " << error << ")";
        log("ERROR", msg.str());
        return false;
    }

    initialized_ = true;
    log("INFO", "OpenVR initialized successfully");
    return true;
}

void OpenVRContext::shutdown()
{
    if (initialized_ && vr_system_)
    {
        vr::VR_Shutdown();
        vr_system_ = nullptr;
        initialized_ = false;
        log("INFO", "OpenVR shut down");
    }
}

void OpenVRContext::updateAllPoses()
{
    if (!vr_system_)
    {
        return;
    }

    vr_system_->GetDeviceToAbsoluteTrackingPose(
        vr::TrackingUniverseStanding,
        0.0f,
        cached_poses_,
        vr::k_unMaxTrackedDeviceCount);

    auto now = std::chrono::high_resolution_clock::now();
    last_update_timestamp_ = std::chrono::duration<double, std::milli>(now.time_since_epoch()).count();
}

const vr::TrackedDevicePose_t& OpenVRContext::getPose(vr::TrackedDeviceIndex_t idx) const
{
    return cached_poses_[idx];
}

std::optional<vr::TrackedDeviceIndex_t> OpenVRContext::findTrackerBySerial(
    const std::string& serial_number)
{
    if (!vr_system_)
    {
        return std::nullopt;
    }

    for (vr::TrackedDeviceIndex_t device_id = 0;
         device_id < vr::k_unMaxTrackedDeviceCount;
         ++device_id)
    {
        if (vr_system_->GetTrackedDeviceClass(device_id) != vr::TrackedDeviceClass_GenericTracker)
        {
            continue;
        }

        if (!vr_system_->IsTrackedDeviceConnected(device_id))
        {
            continue;
        }

        std::array<char, 256> serial{};
        vr::ETrackedPropertyError error;
        vr_system_->GetStringTrackedDeviceProperty(
            device_id,
            vr::Prop_SerialNumber_String,
            serial.data(),
            serial.size(),
            &error);

        if (error != vr::TrackedProp_Success)
        {
            continue;
        }

        if (serial_number == serial.data())
        {
            return device_id;
        }
    }

    return std::nullopt;
}

}

