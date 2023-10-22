#pragma once

#include <Eigen/Dense>

/// @brief Parse input string to double matrix of specific shape and delimiter
/// @param input input string
/// @param R number of rows
/// @param C number of columns
/// @param delimiter delimiter 
/// @return parsed matrix
Eigen::MatrixXd parseMatrixXd(const std::string &input, int R, int C, char delimiter = ' ');
/// @brief Parse input string to double vector of specific length and delimiter
/// @param str input string
/// @param noOfElem length of vector
/// @param delimiter delimiter
/// @return parsed vector
Eigen::VectorXd parseVectorXd(std::string str, int noOfElem, char delimiter = ' ');