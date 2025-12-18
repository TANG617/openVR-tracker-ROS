#include "tracker/pipeline/detector.hpp"
#include <numeric>
#include <sstream>
#include <iomanip>

namespace tracker::pipeline
{

JumpDetector::JumpDetector(const DetectorConfig& config)
    : config_(config)
{
}

void JumpDetector::log(const std::string& level, const std::string& message)
{
    if (log_callback_)
    {
        log_callback_(level, message);
    }
}

DetectResult JumpDetector::detect(const Eigen::Vector3d& position, double timestamp)
{
    if (!config_.enable_jump_detection)
    {
        return DetectResult::kNormal;
    }

    if (state_ == State::kLocked)
    {
        recovery_window_.push_back(position);

        if (recovery_window_.size() >= config_.recovery_window_size)
        {
            last_variance_ = computeVariance();

            if (isWindowStable())
            {
                state_ = State::kNormal;
                prev_position_ = recovery_window_.back();
                prev_timestamp_ = timestamp;
                prev_valid_ = true;
                recovery_window_.clear();

                std::ostringstream msg;
                msg << "[RECOVERED] variance: " << std::scientific << std::setprecision(4)
                    << last_variance_ << " (threshold: " << config_.stability_threshold
                    << "), resuming output";
                log("INFO", msg.str());

                return DetectResult::kRecovered;
            }
            recovery_window_.pop_front();
        }

        // 每10帧输出一次锁定状态日志
        if (recovery_window_.size() % 10 == 0 || recovery_window_.size() == config_.recovery_window_size - 1)
        {
            last_variance_ = computeVariance();
            std::ostringstream msg;
            msg << "[LOCKED] variance: " << std::scientific << std::setprecision(4)
                << last_variance_ << " (threshold: " << config_.stability_threshold
                << "), window: " << recovery_window_.size() << "/" << config_.recovery_window_size;
            log("WARN", msg.str());
        }

        return DetectResult::kLocked;
    }

    if (detectJump(position, timestamp))
    {
        state_ = State::kLocked;
        recovery_window_.clear();
        recovery_window_.push_back(position);

        std::ostringstream msg;
        msg << "[JUMP DETECTED] speed: " << std::fixed << std::setprecision(2)
            << last_speed_ << " m/s (limit: " << config_.jump_speed_limit
            << " m/s), entering LOCKED state";
        log("WARN", msg.str());

        return DetectResult::kJump;
    }

    prev_position_ = position;
    prev_timestamp_ = timestamp;
    prev_valid_ = true;

    return DetectResult::kNormal;
}

void JumpDetector::reset()
{
    state_ = State::kNormal;
    prev_valid_ = false;
    prev_position_ = Eigen::Vector3d::Zero();
    prev_timestamp_ = 0.0;
    recovery_window_.clear();
    last_speed_ = 0.0;
    last_variance_ = 0.0;

    log("INFO", "[RESET] Detector state cleared");
}

bool JumpDetector::detectJump(const Eigen::Vector3d& position, double timestamp)
{
    if (!prev_valid_)
    {
        return false;
    }

    double dt = timestamp - prev_timestamp_;
    if (dt < kMinDt)
    {
        return false;
    }

    double distance = (position - prev_position_).norm();
    last_speed_ = distance / dt * 1000.0;  // 转换为 m/s (timestamp是ms)

    return last_speed_ > config_.jump_speed_limit;
}

bool JumpDetector::isWindowStable() const
{
    if (recovery_window_.size() < 2)
    {
        return false;
    }

    double variance = computeVariance();
    return variance < config_.stability_threshold;
}

double JumpDetector::computeVariance() const
{
    if (recovery_window_.empty())
    {
        return 0.0;
    }

    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& pos : recovery_window_)
    {
        mean += pos;
    }
    mean /= static_cast<double>(recovery_window_.size());

    double variance = 0.0;
    for (const auto& pos : recovery_window_)
    {
        variance += (pos - mean).squaredNorm();
    }
    variance /= static_cast<double>(recovery_window_.size());

    return variance;
}

Detector::Detector(const DetectorConfig& config)
    : config_(config)
    , jump_detector_(config)
{
}

DetectResult Detector::detect(const PoseData& pose)
{
    return jump_detector_.detect(pose.position, pose.timestamp);
}

void Detector::reset()
{
    jump_detector_.reset();
}

void Detector::setConfig(const DetectorConfig& config)
{
    config_ = config;
    jump_detector_.setConfig(config);
}

}


