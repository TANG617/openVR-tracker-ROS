#include <openvr.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <map>

struct TrackerInfo
{
    std::string serial_number;
    vr::TrackedDeviceIndex_t device_id;
};

std::vector<TrackerInfo> scanTrackers(vr::IVRSystem* vr_system)
{
    std::vector<TrackerInfo> trackers;

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
            TrackerInfo info;
            info.serial_number = std::string(serial);
            info.device_id = device_id;
            trackers.push_back(info);
        }
    }

    return trackers;
}

void writeYamlConfig(const std::string& output_path,
                     const std::vector<std::string>& names,
                     const std::map<std::string, std::string>& name_to_sn)
{
    std::ofstream ofs(output_path);
    if (!ofs.is_open())
    {
        std::cerr << "Error: Cannot open file for writing: " << output_path << std::endl;
        return;
    }

    ofs << "tracker:\n";
    ofs << "  trackers:\n";
    ofs << "    ros__parameters:\n";
    ofs << "      publish_rate: 100.0\n";
    ofs << "      parent_frame: \"map\"\n";
    ofs << "      base_link: \"base_link\"\n";
    ofs << "      calibration_file: \"calibration.yaml\"\n";
    ofs << "\n";
    ofs << "      detector:\n";
    ofs << "        enable_jump_detection: true\n";
    ofs << "        jump_speed_limit: 3.0\n";
    ofs << "        recovery_window_size: 10\n";
    ofs << "        stability_threshold: 0.01\n";
    ofs << "\n";
    ofs << "      filter:\n";
    ofs << "        enable: true\n";
    ofs << "        smoothing_alpha: 0.5\n";
    ofs << "        normal_speed_limit: 2.0\n";
    ofs << "\n";

    ofs << "      tracker_names: [";
    for (size_t i = 0; i < names.size(); ++i)
    {
        ofs << "\"" << names[i] << "\"";
        if (i < names.size() - 1) ofs << ", ";
    }
    ofs << "]\n";
    ofs << "\n";
    ofs << "      trackers:\n";

    for (const auto& name : names)
    {
        auto it = name_to_sn.find(name);
        if (it != name_to_sn.end())
        {
            ofs << "        " << name << ":\n";
            ofs << "          serial_number: \"" << it->second << "\"\n";
        }
    }

    ofs.close();
    std::cout << "\nConfiguration written to: " << output_path << std::endl;
}

int main(int argc, char** argv)
{
    std::string output_path = "tracker_params.yaml";
    if (argc > 1)
    {
        output_path = argv[1];
    }

    std::cout << "=== Vive Tracker Configuration Tool ===" << std::endl;
    std::cout << "Output file: " << output_path << std::endl;
    std::cout << "\nInitializing OpenVR..." << std::endl;

    vr::EVRInitError error = vr::VRInitError_None;
    vr::IVRSystem* vr_system = vr::VR_Init(&error, vr::VRApplication_Background);

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

    std::cout << "Scanning for Vive Trackers...\n" << std::endl;

    auto trackers = scanTrackers(vr_system);

    if (trackers.empty())
    {
        std::cout << "No connected Vive Trackers found." << std::endl;
        vr::VR_Shutdown();
        return 1;
    }

    std::cout << "Found " << trackers.size() << " tracker(s):\n" << std::endl;
    for (size_t i = 0; i < trackers.size(); ++i)
    {
        std::cout << "  [" << i << "] " << trackers[i].serial_number
                  << " (device_id: " << trackers[i].device_id << ")" << std::endl;
    }

    std::vector<std::string> names;
    std::map<std::string, std::string> name_to_sn;

    std::cout << "\n--- Configure each tracker ---" << std::endl;
    std::cout << "Enter a name for each tracker (e.g., left, right, head)" << std::endl;
    std::cout << "Press Enter to skip a tracker\n" << std::endl;

    for (size_t i = 0; i < trackers.size(); ++i)
    {
        std::cout << "Tracker [" << trackers[i].serial_number << "] name: ";
        std::string name;
        std::getline(std::cin, name);

        if (name.empty())
        {
            std::cout << "  -> Skipped" << std::endl;
            continue;
        }

        for (auto& c : name)
        {
            if (c == ' ') c = '_';
        }

        names.push_back(name);
        name_to_sn[name] = trackers[i].serial_number;
        std::cout << "  -> Configured as '" << name << "'" << std::endl;
    }

    vr::VR_Shutdown();

    if (names.empty())
    {
        std::cout << "\nNo trackers configured. Exiting." << std::endl;
        return 0;
    }

    std::cout << "\n--- Summary ---" << std::endl;
    for (const auto& name : names)
    {
        std::cout << "  " << name << ": " << name_to_sn[name] << std::endl;
    }

    std::cout << "\nWrite configuration to " << output_path << "? [Y/n]: ";
    std::string confirm;
    std::getline(std::cin, confirm);

    if (confirm.empty() || confirm[0] == 'Y' || confirm[0] == 'y')
    {
        writeYamlConfig(output_path, names, name_to_sn);
    }
    else
    {
        std::cout << "Configuration not saved." << std::endl;
    }

    return 0;
}

