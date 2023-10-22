#pragma once
#include <Eigen/Dense>
#include <mutex>
#include <memory>

/// @brief Hinge connecting aircraft with drives
class Hinge
{
public:
    Hinge() = default;
    Hinge(Eigen::Vector3d axis, double max, double min, double trim);
    Hinge(const Hinge& old);
    Hinge& operator=(const Hinge& old);

    /// @brief set new angle on hinge
    /// @param newValue new angle of hinge
    void updateValue(double newValue);
    /// @brief Get rotattion matrix of orientation change due to hinge
    /// @return rotation matrix
    const Eigen::Matrix3d getRot();

private:
    Eigen::Vector3d axis;
    double max;
    double min;

    std::mutex mtx;
    double value;
    Eigen::Matrix3d rot;
};