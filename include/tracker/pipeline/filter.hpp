#pragma once

#include <functional>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "tracker/pipeline/types.hpp"

namespace tracker::pipeline
{

using FilterLogCallback = std::function<void(const std::string& level, const std::string& message)>;

struct FilterConfig
{
    bool enable{true};
    double smoothing_alpha{0.3};      // 指数平滑系数（越小越平滑）
    double normal_speed_limit{2.0};   // 速度限幅上限 (m/s)
};

class Filter
{
public:
    explicit Filter(const FilterConfig& config = {});

    [[nodiscard]] PoseData apply(const PoseData& input);

    void reset();

    void setConfig(const FilterConfig& config) { config_ = config; }
    void setLogCallback(FilterLogCallback callback) { log_callback_ = std::move(callback); }

    // 获取诊断信息
    [[nodiscard]] double lastInputSpeed() const { return last_input_speed_; }
    [[nodiscard]] double lastOutputSpeed() const { return last_output_speed_; }
    [[nodiscard]] bool wasSpeedClamped() const { return was_speed_clamped_; }

private:
    [[nodiscard]] Eigen::Vector3d clampVelocity(
        const Eigen::Vector3d& current,
        const Eigen::Vector3d& previous,
        double dt);

    void log(const std::string& level, const std::string& message);

    FilterConfig config_;

    PoseData prev_pose_;
    PoseData smoothed_pose_;
    bool initialized_{false};

    // 诊断数据
    double last_input_speed_{0.0};
    double last_output_speed_{0.0};
    bool was_speed_clamped_{false};

    FilterLogCallback log_callback_;

    static constexpr double kMinDt = 0.001;
};

}


