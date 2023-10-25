#pragma once

#include <Eigen/Dense>

class Aircraft;

/// @brief Aircraft's control surfaces
class ControlSurfaces
{
public:
    ControlSurfaces();
    /// @brief Constructor
    /// @param noOfSurfaces number of independent surfaces 
    /// @param matrix cofficients matrix
    /// @param min vector of min angles
    /// @param max vector of max angles
    /// @param trim vector of trim angles
    ControlSurfaces(int noOfSurfaces, Eigen::Matrix<double,6,-1> matrix,
        Eigen::VectorXd min, Eigen::VectorXd max, Eigen::VectorXd trim);

    Eigen::Vector<double,6> getCofficients() const;
    bool setValues(Eigen::VectorXd new_values);
    void restoreTrim();
    int getNoOfSurface() const {return noOfSurfaces;}
    Eigen::VectorXd getValues() const {return values;}

private:
    int noOfSurfaces;
    Eigen::Matrix<double,6,-1> matrix;
    Eigen::VectorXd min;
    Eigen::VectorXd max;
    Eigen::VectorXd trim;
    Eigen::VectorXd values;
};