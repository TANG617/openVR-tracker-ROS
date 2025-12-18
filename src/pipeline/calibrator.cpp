#include "tracker/pipeline/calibrator.hpp"
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace tracker::pipeline
{

Calibrator::Calibrator()
{
    Eigen::Matrix3d default_world_rot;
    default_world_rot << -1, 0, 0,
                          0, 0, 1,
                          0, 1, 0;
    world_rot_mat_.block<3, 3>(0, 0) = default_world_rot;

    tracker2wrist_mat_(2, 3) = -0.11;

    default_mounting_ << 0, -1, 0,
                        -1, 0, 0,
                         0, 0, -1;
}

Calibrator::Calibrator(const CalibratorConfig& config)
    : Calibrator()
{
    loadConfig(config);
}

void Calibrator::loadFromYaml(const std::string& yaml_path)
{
    YAML::Node root = YAML::LoadFile(yaml_path);

    if (root["world_rot"])
    {
        auto values = root["world_rot"].as<std::vector<double>>();
        if (values.size() >= 9)
        {
            Eigen::Matrix3d rot;
            rot << values[0], values[1], values[2],
                   values[3], values[4], values[5],
                   values[6], values[7], values[8];
            setWorldRotation(rot);
        }
    }

    if (root["tracker2wrist_z_offset"])
    {
        setTrackerToWristOffset(root["tracker2wrist_z_offset"].as<double>());
    }

    if (root["mounting"])
    {
        for (auto it = root["mounting"].begin(); it != root["mounting"].end(); ++it)
        {
            std::string name = it->first.as<std::string>();
            auto values = it->second.as<std::vector<double>>();
            if (values.size() >= 9)
            {
                Eigen::Matrix3d rot;
                rot << values[0], values[1], values[2],
                       values[3], values[4], values[5],
                       values[6], values[7], values[8];
                setMountingCalibration(name, rot);
            }
        }
    }

    if (root["init_tcp"])
    {
        for (auto it = root["init_tcp"].begin(); it != root["init_tcp"].end(); ++it)
        {
            std::string name = it->first.as<std::string>();
            auto node = it->second;

            Eigen::Vector3d pos = Eigen::Vector3d::Zero();
            Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();

            if (node["position"])
            {
                auto p = node["position"].as<std::vector<double>>();
                if (p.size() >= 3)
                {
                    pos << p[0], p[1], p[2];
                }
            }

            if (node["orientation"])
            {
                auto o = node["orientation"].as<std::vector<double>>();
                if (o.size() >= 9)
                {
                    rot << o[0], o[1], o[2],
                           o[3], o[4], o[5],
                           o[6], o[7], o[8];
                }
            }

            setInitTcpPose(name, pos, rot);
        }
    }
}

void Calibrator::loadConfig(const CalibratorConfig& config)
{
    setWorldRotation(config.world_rot);
    setTrackerToWristOffset(config.tracker2wrist_z_offset);

    for (const auto& [name, rot] : config.mounting)
    {
        setMountingCalibration(name, rot);
    }

    for (const auto& [name, pos] : config.init_tcp_position)
    {
        Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
        auto it = config.init_tcp_orientation.find(name);
        if (it != config.init_tcp_orientation.end())
        {
            rot = it->second;
        }
        setInitTcpPose(name, pos, rot);
    }
}

void Calibrator::setWorldRotation(const Eigen::Matrix3d& rot)
{
    world_rot_mat_.block<3, 3>(0, 0) = rot;
}

void Calibrator::setMountingCalibration(const std::string& name, const Eigen::Matrix3d& rot)
{
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat.block<3, 3>(0, 0) = rot;
    mounting_matrices_[name] = mat;
}

void Calibrator::setTrackerToWristOffset(double z_offset)
{
    tracker2wrist_mat_(2, 3) = z_offset;
}

void Calibrator::setInitTcpPose(const std::string& name, const Eigen::Vector3d& pos, const Eigen::Matrix3d& rot)
{
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat.block<3, 3>(0, 0) = rot;
    mat.block<3, 1>(0, 3) = pos;
    init_tcp_matrices_[name] = mat;
}

Eigen::Matrix4d Calibrator::transform(
    const Eigen::Matrix4d& raw_pose,
    const std::string& tracker_name) const
{
    Eigen::Matrix4d mounting_mat = getMountingMatrix(tracker_name);

    Eigen::Matrix4d T_tcp_steamvr = raw_pose * mounting_mat;
    Eigen::Matrix4d T_tcp_robot = world_rot_mat_ * T_tcp_steamvr;

    return T_tcp_robot * tracker2wrist_mat_;
}

Eigen::Matrix4d Calibrator::computeTargetPose(
    const Eigen::Matrix4d& current_tracker,
    const Eigen::Matrix4d& init_tracker,
    const std::string& tracker_name) const
{
    Eigen::Matrix4d init_tcp = getInitTcpMatrix(tracker_name);

    Eigen::Matrix3d delta_R = current_tracker.block<3, 3>(0, 0) *
                              init_tracker.block<3, 3>(0, 0).transpose();

    Eigen::Vector3d delta_pos = current_tracker.block<3, 1>(0, 3) -
                                init_tracker.block<3, 1>(0, 3);

    Eigen::Matrix4d target = Eigen::Matrix4d::Identity();
    target.block<3, 3>(0, 0) = delta_R * init_tcp.block<3, 3>(0, 0);
    target.block<3, 1>(0, 3) = delta_pos + init_tcp.block<3, 1>(0, 3);

    return target;
}

Eigen::Matrix4d Calibrator::initTcpPose(const std::string& name) const
{
    return getInitTcpMatrix(name);
}

void Calibrator::updateInitTcpPose(const std::string& name, const Eigen::Matrix4d& pose)
{
    init_tcp_matrices_[name] = pose;
}

Eigen::Matrix4d Calibrator::getMountingMatrix(const std::string& name) const
{
    auto it = mounting_matrices_.find(name);
    if (it != mounting_matrices_.end())
    {
        return it->second;
    }

    Eigen::Matrix4d default_mat = Eigen::Matrix4d::Identity();
    default_mat.block<3, 3>(0, 0) = default_mounting_;
    return default_mat;
}

Eigen::Matrix4d Calibrator::getInitTcpMatrix(const std::string& name) const
{
    auto it = init_tcp_matrices_.find(name);
    if (it != init_tcp_matrices_.end())
    {
        return it->second;
    }

    return Eigen::Matrix4d::Identity();
}

}


