#pragma once

#include <string>
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "tracker/pipeline/types.hpp"

namespace tracker::pipeline
{

struct CalibratorConfig
{
    Eigen::Matrix3d world_rot{Eigen::Matrix3d::Identity()};
    double tracker2wrist_z_offset{-0.11};
    std::map<std::string, Eigen::Matrix3d> mounting;
    std::map<std::string, Eigen::Vector3d> init_tcp_position;
    std::map<std::string, Eigen::Matrix3d> init_tcp_orientation;
};

class Calibrator
{
public:
    Calibrator();
    explicit Calibrator(const CalibratorConfig& config);

    void loadFromYaml(const std::string& yaml_path);
    void loadConfig(const CalibratorConfig& config);

    void setWorldRotation(const Eigen::Matrix3d& rot);
    void setMountingCalibration(const std::string& name, const Eigen::Matrix3d& rot);
    void setTrackerToWristOffset(double z_offset);
    void setInitTcpPose(const std::string& name, const Eigen::Vector3d& pos, const Eigen::Matrix3d& rot);

    [[nodiscard]] Eigen::Matrix4d transform(
        const Eigen::Matrix4d& raw_pose, 
        const std::string& tracker_name) const;

    [[nodiscard]] Eigen::Matrix4d computeTargetPose(
        const Eigen::Matrix4d& current_tracker,
        const Eigen::Matrix4d& init_tracker,
        const std::string& tracker_name) const;

    [[nodiscard]] Eigen::Matrix4d initTcpPose(const std::string& name) const;

    void updateInitTcpPose(const std::string& name, const Eigen::Matrix4d& pose);

private:
    [[nodiscard]] Eigen::Matrix4d getMountingMatrix(const std::string& name) const;
    [[nodiscard]] Eigen::Matrix4d getInitTcpMatrix(const std::string& name) const;

    Eigen::Matrix4d world_rot_mat_{Eigen::Matrix4d::Identity()};
    Eigen::Matrix4d tracker2wrist_mat_{Eigen::Matrix4d::Identity()};
    
    std::map<std::string, Eigen::Matrix4d> mounting_matrices_;
    std::map<std::string, Eigen::Matrix4d> init_tcp_matrices_;

    Eigen::Matrix3d default_mounting_;
};

}


