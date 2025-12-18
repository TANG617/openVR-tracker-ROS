#include "tracker/trackers_node.hpp"
#include "tracker/openvr_context.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace tracker
{

TrackersNode::TrackersNode()
    : Node("trackers_node")
{
    data_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    this->declare_parameter("publish_rate", 100.0);
    this->declare_parameter("parent_frame", "map");
    this->declare_parameter("base_link", "base_link");
    this->declare_parameter("calibration_file", "calibration.yaml");
    this->declare_parameter("tracker_names", std::vector<std::string>{});

    this->declare_parameter("detector.enable_jump_detection", true);
    this->declare_parameter("detector.jump_speed_limit", 5.0);
    this->declare_parameter("detector.recovery_window_size", 20);
    this->declare_parameter("detector.stability_threshold", 0.0001);

    this->declare_parameter("filter.enable", true);
    this->declare_parameter("filter.smoothing_alpha", 0.3);
    this->declare_parameter("filter.normal_speed_limit", 2.0);

    double publish_rate = this->get_parameter("publish_rate").as_double();
    parent_frame_ = this->get_parameter("parent_frame").as_string();
    base_link_ = this->get_parameter("base_link").as_string();
    calibration_file_ = this->get_parameter("calibration_file").as_string();

    detector_config_.enable_jump_detection = this->get_parameter("detector.enable_jump_detection").as_bool();
    detector_config_.jump_speed_limit = this->get_parameter("detector.jump_speed_limit").as_double();
    detector_config_.recovery_window_size = static_cast<size_t>(
        this->get_parameter("detector.recovery_window_size").as_int());
    detector_config_.stability_threshold = this->get_parameter("detector.stability_threshold").as_double();

    filter_config_.enable = this->get_parameter("filter.enable").as_bool();
    filter_config_.smoothing_alpha = this->get_parameter("filter.smoothing_alpha").as_double();
    filter_config_.normal_speed_limit = this->get_parameter("filter.normal_speed_limit").as_double();

    loadCalibration();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if (!loadTrackerConfigs())
    {
        throw std::runtime_error("No tracker configurations found");
    }

    initializeTrackers();

    reset_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/reset_tracker_origin",
        std::bind(&TrackersNode::resetOriginCallback, this,
                 std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_callback_group_);

    auto interval = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
    timer_ = this->create_wall_timer(interval,
                                     std::bind(&TrackersNode::timerCallback, this),
                                     data_callback_group_);

    RCLCPP_INFO(this->get_logger(), "TrackersNode started: %zu trackers @ %.1f Hz",
                tracker_instances_.size(), publish_rate);
}

void TrackersNode::loadCalibration()
{
    calibrator_ = std::make_shared<pipeline::Calibrator>();

    try
    {
        std::string package_share = ament_index_cpp::get_package_share_directory("tracker");
        std::string full_path = package_share + "/config/" + calibration_file_;
        calibrator_->loadFromYaml(full_path);
        RCLCPP_INFO(this->get_logger(), "Loaded calibration from: %s", full_path.c_str());
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to load calibration file: %s, using defaults", e.what());
    }
}

bool TrackersNode::loadTrackerConfigs()
{
    auto names = this->get_parameter("tracker_names").as_string_array();

    if (names.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No tracker_names configured");
        return false;
    }

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = data_callback_group_;

    for (const auto& name : names)
    {
        std::string prefix = "trackers." + name + ".";

        this->declare_parameter(prefix + "serial_number", "");
        this->declare_parameter(prefix + "raw_topic_name", "/hal/tracker/htc/" + name + "/pose_raw");
        this->declare_parameter(prefix + "calib_topic_name", "/hal/tracker/htc/" + name + "/calib_target_pose");
        this->declare_parameter(prefix + "frame_id", "vive_" + name);
        this->declare_parameter(prefix + "tcp_topic", "");

        TrackerConfig config;
        config.name = name;
        config.serial_number = this->get_parameter(prefix + "serial_number").as_string();
        config.raw_topic_name = this->get_parameter(prefix + "raw_topic_name").as_string();
        config.calib_topic_name = this->get_parameter(prefix + "calib_topic_name").as_string();
        config.frame_id = this->get_parameter(prefix + "frame_id").as_string();

        if (config.serial_number.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Tracker '%s' has no serial_number, skipping", name.c_str());
            continue;
        }

        std::string tcp_topic = this->get_parameter(prefix + "tcp_topic").as_string();
        if (!tcp_topic.empty())
        {
            current_tcp_poses_[name] = calibrator_->initTcpPose(name);
            tcp_received_[name] = false;

            tcp_subs_[name] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                tcp_topic, 10,
                [this, name](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    tcpPoseCallback(msg, name);
                },
                sub_options);
        }

        tracker_configs_.push_back(config);
    }

    return !tracker_configs_.empty();
}

void TrackersNode::initializeTrackers()
{
    // 输出配置参数日志
    RCLCPP_INFO(this->get_logger(),
        "Detector config: jump_speed_limit=%.2f m/s, recovery_window=%zu frames, stability_threshold=%.6f m²",
        detector_config_.jump_speed_limit,
        detector_config_.recovery_window_size,
        detector_config_.stability_threshold);
    RCLCPP_INFO(this->get_logger(),
        "Filter config: smoothing_alpha=%.2f, normal_speed_limit=%.2f m/s",
        filter_config_.smoothing_alpha,
        filter_config_.normal_speed_limit);

    for (const auto& config : tracker_configs_)
    {
        TrackerInstance instance;
        instance.config = config;

        // 创建日志回调（用于 reader, detector, filter）
        auto log_callback = [this, name = config.name](const std::string& level, const std::string& message) {
            std::string full_msg = "[" + name + "] " + message;
            if (level == "DEBUG") RCLCPP_DEBUG(this->get_logger(), "%s", full_msg.c_str());
            else if (level == "INFO") RCLCPP_INFO(this->get_logger(), "%s", full_msg.c_str());
            else if (level == "WARN") RCLCPP_WARN(this->get_logger(), "%s", full_msg.c_str());
            else if (level == "ERROR") RCLCPP_ERROR(this->get_logger(), "%s", full_msg.c_str());
        };

        instance.reader = std::make_unique<TrackerReader>(config.serial_number, log_callback);

        instance.validator = std::make_unique<pipeline::Validator>();

        instance.detector = std::make_unique<pipeline::Detector>(detector_config_);
        instance.detector->setLogCallback(log_callback);

        instance.filter = std::make_unique<pipeline::Filter>(filter_config_);
        instance.filter->setLogCallback(log_callback);

        instance.raw_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            config.raw_topic_name, 10);
        instance.calib_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            config.calib_topic_name, 10);

        tracker_instances_.push_back(std::move(instance));

        RCLCPP_INFO(this->get_logger(), "Initialized tracker '%s' (SN: %s)",
            config.name.c_str(), config.serial_number.c_str());
    }

    if (tracker_instances_.empty())
    {
        throw std::runtime_error("No tracker instances created");
    }
}

void TrackersNode::timerCallback()
{
    OpenVRContext::instance().updateAllPoses();

    for (auto& instance : tracker_instances_)
    {
        TrackerPose pose;

        if (instance.reader->getPose(pose))
        {
            processAndPublish(instance, pose);
        }
    }
}

void TrackersNode::processAndPublish(TrackerInstance& instance, const TrackerPose& pose)
{
    auto validate_result = instance.validator->validate(pose);
    if (validate_result != pipeline::ValidateResult::kValid)
    {
        return;
    }

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 4>(0, 0) = pose.matrix.cast<double>();

    auto stamp = this->now();
    const std::string& name = instance.config.name;

    Eigen::Vector3d raw_position = T.block<3, 1>(0, 3);
    Eigen::Matrix3d raw_rotation = T.block<3, 3>(0, 0);
    Eigen::Quaterniond raw_q(raw_rotation);
    raw_q.normalize();

    publishPose(instance.raw_pub, stamp, base_link_, raw_position, raw_q);
    broadcastTf(stamp, base_link_, instance.config.frame_id + "_raw", raw_position, raw_q);

    T = calibrator_->transform(T, name);

    if (!instance.initialized)
    {
        instance.init_tracker_pose = T;
        instance.initialized = true;
    }

    Eigen::Matrix4d target = calibrator_->computeTargetPose(T, instance.init_tracker_pose, name);

    Eigen::Vector3d position = target.block<3, 1>(0, 3);
    Eigen::Matrix3d rotation = target.block<3, 3>(0, 0);
    Eigen::Quaterniond q(rotation);
    q.normalize();

    pipeline::PoseData pose_data;
    pose_data.position = position;
    pose_data.orientation = q;
    pose_data.timestamp = pose.timestamp;
    pose_data.valid = true;

    auto detect_result = instance.detector->detect(pose_data);
    if (detect_result == pipeline::DetectResult::kJump ||
        detect_result == pipeline::DetectResult::kLocked)
    {
        return;
    }

    pipeline::PoseData filtered = instance.filter->apply(pose_data);

    publishPose(instance.calib_pub, stamp, parent_frame_, filtered.position, filtered.orientation);
    broadcastTf(stamp, base_link_, instance.config.frame_id, filtered.position, filtered.orientation);
}

void TrackersNode::tcpPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const std::string& name)
{
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose(0, 3) = msg->pose.position.x;
    pose(1, 3) = msg->pose.position.y;
    pose(2, 3) = msg->pose.position.z;

    Eigen::Quaterniond q(msg->pose.orientation.w,
                        msg->pose.orientation.x,
                        msg->pose.orientation.y,
                        msg->pose.orientation.z);
    pose.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();

    current_tcp_poses_[name] = pose;

    if (!tcp_received_[name])
    {
        tcp_received_[name] = true;
        RCLCPP_INFO(this->get_logger(), "First TCP pose received for '%s'", name.c_str());
    }
}

void TrackersNode::resetOriginCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    for (const auto& [name, received] : tcp_received_)
    {
        if (received)
        {
            calibrator_->updateInitTcpPose(name, current_tcp_poses_[name]);
        }
    }

    for (auto& instance : tracker_instances_)
    {
        instance.detector->reset();
        instance.filter->reset();

        TrackerPose tracker_pose;
        if (instance.reader && instance.reader->getPose(tracker_pose))
        {
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3, 4>(0, 0) = tracker_pose.matrix.cast<double>();

            instance.init_tracker_pose = calibrator_->transform(T, instance.config.name);
            instance.initialized = true;
        }
        else
        {
            instance.initialized = false;
            instance.init_tracker_pose = Eigen::Matrix4d::Identity();
        }
    }

    response->success = true;
    response->message = "Tracker origins reset successfully.";
    RCLCPP_INFO(this->get_logger(), "Tracker origin reset completed");
}

void TrackersNode::publishPose(
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& pub,
    const rclcpp::Time& stamp,
    const std::string& frame_id,
    const Eigen::Vector3d& pos,
    const Eigen::Quaterniond& ori)
{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.pose.position.x = pos.x();
    msg.pose.position.y = pos.y();
    msg.pose.position.z = pos.z();
    msg.pose.orientation.x = ori.x();
    msg.pose.orientation.y = ori.y();
    msg.pose.orientation.z = ori.z();
    msg.pose.orientation.w = ori.w();
    pub->publish(msg);
}

void TrackersNode::broadcastTf(
    const rclcpp::Time& stamp,
    const std::string& parent_frame,
    const std::string& child_frame,
    const Eigen::Vector3d& pos,
    const Eigen::Quaterniond& ori)
{
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = parent_frame;
    tf_msg.child_frame_id = child_frame;
    tf_msg.transform.translation.x = pos.x();
    tf_msg.transform.translation.y = pos.y();
    tf_msg.transform.translation.z = pos.z();
    tf_msg.transform.rotation.x = ori.x();
    tf_msg.transform.rotation.y = ori.y();
    tf_msg.transform.rotation.z = ori.z();
    tf_msg.transform.rotation.w = ori.w();
    tf_broadcaster_->sendTransform(tf_msg);
}

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<tracker::TrackersNode>();
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("trackers_node"), "Error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
