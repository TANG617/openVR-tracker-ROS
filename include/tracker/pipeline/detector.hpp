#pragma once

#include <deque>
#include <functional>
#include <string>
#include <Eigen/Core>
#include "tracker/pipeline/types.hpp"

namespace tracker::pipeline
{

using DetectorLogCallback = std::function<void(const std::string& level, const std::string& message)>;

struct DetectorConfig
{
    bool enable_jump_detection{true};
    double jump_speed_limit{5.0};         // 速度超过此值判定为跳跃 (m/s)
    size_t recovery_window_size{20};      // 恢复所需的稳定帧数
    double stability_threshold{0.0001};   // 位置方差阈值 (m²)，标准差约1cm
};

class JumpDetector
{
public:
    enum class State { kNormal, kLocked };

    explicit JumpDetector(const DetectorConfig& config = {});

    [[nodiscard]] DetectResult detect(const Eigen::Vector3d& position, double timestamp);

    void reset();

    [[nodiscard]] State state() const { return state_; }
    [[nodiscard]] bool isLocked() const { return state_ == State::kLocked; }

    void setConfig(const DetectorConfig& config) { config_ = config; }
    void setLogCallback(DetectorLogCallback callback) { log_callback_ = std::move(callback); }

    // 获取诊断信息
    [[nodiscard]] double lastSpeed() const { return last_speed_; }
    [[nodiscard]] double lastVariance() const { return last_variance_; }
    [[nodiscard]] size_t recoveryProgress() const { return recovery_window_.size(); }

private:
    [[nodiscard]] bool detectJump(const Eigen::Vector3d& position, double timestamp);
    [[nodiscard]] bool isWindowStable() const;
    [[nodiscard]] double computeVariance() const;
    void log(const std::string& level, const std::string& message);

    DetectorConfig config_;
    State state_{State::kNormal};

    Eigen::Vector3d prev_position_{Eigen::Vector3d::Zero()};
    double prev_timestamp_{0.0};
    bool prev_valid_{false};

    std::deque<Eigen::Vector3d> recovery_window_;

    // 诊断数据
    double last_speed_{0.0};
    double last_variance_{0.0};

    DetectorLogCallback log_callback_;

    static constexpr double kMinDt = 0.001;
};

class Detector
{
public:
    explicit Detector(const DetectorConfig& config = {});

    [[nodiscard]] DetectResult detect(const PoseData& pose);

    void reset();

    [[nodiscard]] bool isLocked() const { return jump_detector_.isLocked(); }

    void setConfig(const DetectorConfig& config);
    void setLogCallback(DetectorLogCallback callback) { jump_detector_.setLogCallback(std::move(callback)); }

    // 获取诊断信息
    [[nodiscard]] double lastSpeed() const { return jump_detector_.lastSpeed(); }
    [[nodiscard]] double lastVariance() const { return jump_detector_.lastVariance(); }
    [[nodiscard]] size_t recoveryProgress() const { return jump_detector_.recoveryProgress(); }

private:
    DetectorConfig config_;
    JumpDetector jump_detector_;
};

}


