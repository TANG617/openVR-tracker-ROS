#pragma once

#include <functional>
#include <string>
#include <Eigen/Core>

namespace tracker
{

using LogCallback = std::function<void(const std::string&, const std::string&)>;

enum class FilterResult
{
    kNormal = 0,
    kClamped = 1,
    kJump = 2
};

struct TrackerPose
{
    int device_id{-1};
    Eigen::Matrix<float, 3, 4, Eigen::RowMajor> matrix{
        Eigen::Matrix<float, 3, 4, Eigen::RowMajor>::Zero()};
    double timestamp{0.0};
    bool is_valid{false};
};

}

