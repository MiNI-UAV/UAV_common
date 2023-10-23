#pragma once
#include <Eigen/Dense>

/// @brief Aerodynamic cofficient
struct AeroCofficients
{
    double S; 
    double d;
    double eAR;
    Eigen::Vector<double,6> C0;
    Eigen::Matrix<double,6,3> Cpqr;
    Eigen::Matrix<double,6,4> Cab;
    double stallLimit;
};
