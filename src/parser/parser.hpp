#pragma once

#include <Eigen/Dense>

Eigen::MatrixXd parseMatrixXd(const std::string &input, int R, int C, char delimiter = ' ');
Eigen::VectorXd parseVectorXd(std::string str, int noOfElem, char delimiter = ' ');