#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "tracker/types.hpp"
#include "tracker/tracker_reader.hpp"
#include "tracker/pipeline/types.hpp"
#include "tracker/pipeline/validator.hpp"
#include "tracker/pipeline/detector.hpp"
#include "tracker/pipeline/calibrator.hpp"
#include "tracker/pipeline/filter.hpp"

namespace tracker
{

struct TrackerConfig
{
    std::string name;
    std::string serial_number;
    std::string raw_topic_name;
    std::string calib_topic_name;
    std::string frame_id;
};

struct TrackerInstance
{
    TrackerConfig config;
    std::unique_ptr<TrackerReader> reader;
    std::unique_ptr<pipeline::Validator> validator;
    std::unique_ptr<pipeline::Detector> detector;
    std::unique_ptr<pipeline::Filter> filter;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr raw_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr calib_pub;

    bool initialized{false};
    Eigen::Matrix4d init_tracker_pose{Eigen::Matrix4d::Identity()};

    TrackerInstance() = default;
    TrackerInstance(const TrackerInstance&) = delete;
    TrackerInstance& operator=(const TrackerInstance&) = delete;
    TrackerInstance(TrackerInstance&&) = default;
    TrackerInstance& operator=(TrackerInstance&&) = default;
};

class TrackersNode : public rclcpp::Node
{
public:
    TrackersNode();
    ~TrackersNode() = default;

private:
    [[nodiscard]] bool loadTrackerConfigs();
    void loadCalibration();
    void initializeTrackers();
    void timerCallback();
    void processAndPublish(TrackerInstance& instance, const TrackerPose& pose);

    void tcpPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const std::string& name);
    void resetOriginCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void publishPose(
        const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& pub,
        const rclcpp::Time& stamp,
        const std::string& frame_id,
        const Eigen::Vector3d& pos,
        const Eigen::Quaterniond& ori);

    void broadcastTf(
        const rclcpp::Time& stamp,
        const std::string& parent_frame,
        const std::string& child_frame,
        const Eigen::Vector3d& pos,
        const Eigen::Quaterniond& ori);

    std::vector<TrackerConfig> tracker_configs_;
    std::vector<TrackerInstance> tracker_instances_;
    std::shared_ptr<pipeline::Calibrator> calibrator_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

    rclcpp::CallbackGroup::SharedPtr data_callback_group_;
    rclcpp::CallbackGroup::SharedPtr service_callback_group_;

    std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> tcp_subs_;
    std::map<std::string, Eigen::Matrix4d> current_tcp_poses_;
    std::map<std::string, bool> tcp_received_;

    std::string parent_frame_;
    std::string base_link_;
    std::string calibration_file_;

    pipeline::DetectorConfig detector_config_;
    pipeline::FilterConfig filter_config_;
};

}

