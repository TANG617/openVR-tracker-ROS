#include "tracker/pipeline/filter.hpp"
#include <cmath>
#include <sstream>
#include <iomanip>

namespace tracker::pipeline
{

Filter::Filter(const FilterConfig& config)
    : config_(config)
{
}

void Filter::log(const std::string& level, const std::string& message)
{
    if (log_callback_)
    {
        log_callback_(level, message);
    }
}

PoseData Filter::apply(const PoseData& input)
{
    if (!config_.enable)
    {
        return input;
    }

    PoseData output = input;

    if (!initialized_)
    {
        prev_pose_ = input;
        smoothed_pose_ = input;
        initialized_ = true;
        log("DEBUG", "[FILTER] Initialized with first pose");
        return output;
    }

    double dt = std::abs(input.timestamp - prev_pose_.timestamp);
    if (dt < kMinDt)
    {
        dt = kMinDt;
    }

    // 转换为秒
    double dt_sec = dt / 1000.0;

    Eigen::Vector3d clamped_pos = clampVelocity(input.position, prev_pose_.position, dt_sec);

    prev_pose_.position = clamped_pos;
    prev_pose_.orientation = input.orientation;
    prev_pose_.timestamp = input.timestamp;

    smoothed_pose_.position = config_.smoothing_alpha * clamped_pos +
                              (1.0 - config_.smoothing_alpha) * smoothed_pose_.position;

    smoothed_pose_.orientation = smoothed_pose_.orientation.slerp(
        config_.smoothing_alpha, input.orientation);
    smoothed_pose_.orientation.normalize();

    output.position = smoothed_pose_.position;
    output.orientation = smoothed_pose_.orientation;

    // 计算输出速度用于诊断
    static Eigen::Vector3d last_output_pos = output.position;
    static double last_output_time = input.timestamp;
    double output_dt = (input.timestamp - last_output_time) / 1000.0;
    if (output_dt > kMinDt)
    {
        last_output_speed_ = (output.position - last_output_pos).norm() / output_dt;
    }
    last_output_pos = output.position;
    last_output_time = input.timestamp;

    return output;
}

void Filter::reset()
{
    initialized_ = false;
    prev_pose_ = PoseData{};
    smoothed_pose_ = PoseData{};
    last_input_speed_ = 0.0;
    last_output_speed_ = 0.0;
    was_speed_clamped_ = false;

    log("INFO", "[FILTER RESET] Filter state cleared");
}

Eigen::Vector3d Filter::clampVelocity(
    const Eigen::Vector3d& current,
    const Eigen::Vector3d& previous,
    double dt)
{
    Eigen::Vector3d delta = current - previous;
    double dist = delta.norm();

    if (dist < 1e-6)
    {
        last_input_speed_ = 0.0;
        was_speed_clamped_ = false;
        return current;
    }

    double speed = dist / dt;
    last_input_speed_ = speed;

    if (speed > config_.normal_speed_limit)
    {
        was_speed_clamped_ = true;

        std::ostringstream msg;
        msg << "[FILTER CLAMPED] speed: " << std::fixed << std::setprecision(2)
            << speed << " -> " << config_.normal_speed_limit << " m/s";
        log("DEBUG", msg.str());

        Eigen::Vector3d direction = delta.normalized();
        double clamped_dist = config_.normal_speed_limit * dt;
        return previous + direction * clamped_dist;
    }

    was_speed_clamped_ = false;
    return current;
}

}


