#include "tracker/steamvr_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

class SteamVRNode : public rclcpp::Node
{
public:
    SteamVRNode() : Node("steamvr")
    {
        RCLCPP_INFO(this->get_logger(), "Starting SteamVR...");

        steamvr_manager_ = std::make_unique<tracker::SteamVRManager>(
            [this](const std::string& level, const std::string& message) {
                if (level == "INFO") RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
                else if (level == "WARN") RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
                else if (level == "ERROR") RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
            });

        if (!steamvr_manager_->start())
        {
            throw std::runtime_error("Failed to start SteamVR");
        }

        RCLCPP_INFO(this->get_logger(), "SteamVR started successfully");
    }

private:
    std::unique_ptr<tracker::SteamVRManager> steamvr_manager_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<SteamVRNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("steamvr"), "Error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}

