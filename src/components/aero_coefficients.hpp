#pragma once
#include <Eigen/Dense>

/// @brief Aerodynamic coefficient
struct AeroCoefficients
{
    double S; 
    double d;
    double eAR;
    Eigen::Vector<double,6> C0;
    Eigen::Matrix<double,6,3> Cpqr;
    Eigen::Matrix<double,6,4> Cab;
    double stallLimit;
};
