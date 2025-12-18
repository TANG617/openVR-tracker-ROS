#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace tracker::pipeline
{

enum class ValidateResult
{
    kValid = 0,
    kInvalidPose = 1,
    kDisconnected = 2
};

enum class DetectResult
{
    kNormal = 0,
    kJump = 1,
    kLocked = 2,
    kRecovered = 3
};

struct PoseData
{
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    double timestamp{0.0};
    bool valid{false};
};

}


