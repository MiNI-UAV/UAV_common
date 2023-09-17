#pragma once
#include <Eigen/Dense>

struct SensorParams
{
    std::string name;
    double sd;
    Eigen::Vector3d bias;
    double refreshTime;
};

struct AHRSParams
{
    std::string type;
    double alpha;
    double Q;
    double R;
};

struct EKFScalers
{
    double predictScaler;
    double updateScaler;
    double baroScaler;
    double zScaler;
};