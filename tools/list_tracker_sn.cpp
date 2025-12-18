#include <openvr.h>
#include <iostream>
#include <vector>
#include <string>

int main(int argc, char **argv)
{
    std::cout << "Initializing OpenVR..." << std::endl;

    vr::EVRInitError error = vr::VRInitError_None;
    vr::IVRSystem *vr_system = vr::VR_Init(&error, vr::VRApplication_Background);

    if (error != vr::VRInitError_None && error != vr::VRInitError_Init_AlreadyRunning)
    {
        std::cerr << "Error: Failed to initialize OpenVR: "
                  << vr::VR_GetVRInitErrorAsEnglishDescription(error) << std::endl;
        return 1;
    }

    if (!vr_system)
    {
        std::cerr << "Error: OpenVR system is null after initialization" << std::endl;
        return 1;
    }

    std::cout << "Scanning for Vive Trackers..." << std::endl;

    std::vector<std::string> tracker_serials;

    for (vr::TrackedDeviceIndex_t device_id = 0;
         device_id < vr::k_unMaxTrackedDeviceCount;
         ++device_id)
    {
        if (vr_system->GetTrackedDeviceClass(device_id) != vr::TrackedDeviceClass_GenericTracker)
        {
            continue;
        }

        if (!vr_system->IsTrackedDeviceConnected(device_id))
        {
            continue;
        }

        char serial[256];
        vr::ETrackedPropertyError prop_error;
        vr_system->GetStringTrackedDeviceProperty(
            device_id,
            vr::Prop_SerialNumber_String,
            serial,
            sizeof(serial),
            &prop_error);

        if (prop_error == vr::TrackedProp_Success)
        {
            tracker_serials.push_back(std::string(serial));
        }
    }

    std::cout << "\n";
    if (tracker_serials.empty())
    {
        std::cout << "No connected Vive Trackers found" << std::endl;
    }
    else
    {
        std::cout << "Connected Vive Trackers:" << std::endl;
        for (const auto &serial : tracker_serials)
        {
            std::cout << serial << std::endl;
        }
    }

    vr::VR_Shutdown();

    return 0;
}
