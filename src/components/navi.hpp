#pragma once
#include <Eigen/Dense>

/// @brief Base parameters of a sensor
struct SensorParams
{
    std::string name;
    double sd;
    Eigen::Vector3d bias;
    double refreshTime;
};

/// @brief AHRS parameters
struct AHRSParams
{
    std::string type;
    double alpha;
    double Q;
    double R;
};

/// @brief Scalers for EKF
struct EKFScalers
{
    double predictScaler;
    double updateScaler;
    double baroScaler;
    double zScaler;
};