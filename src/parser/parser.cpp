#include "parser.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <sstream>

Eigen::MatrixXd parseMatrixXd(const std::string &input, int R, int C, char delimiter)
{
    std::istringstream iss(input);
    std::string token;
    Eigen::MatrixXd matrix;
    matrix.setZero(R,C);
    int i = 0;
    while (std::getline(iss, token, delimiter)) {
        try {
            matrix(i/C,i%C) = std::stod(token);
            i++;
        } catch (const std::exception& e) {
            std::cerr << "Invalid input: " << token << std::endl;
            return Eigen::MatrixXd::Zero(R,C);
        }
    }
    if(i != R*C)
    {
        std::cerr << "Can not parse matrix, got " << i << " values" << std::endl;
        return Eigen::MatrixXd::Zero(R,C);
    }
    return matrix;
}

Eigen::VectorXd parseVectorXd(std::string str, int noOfElem, char delimiter)
{
    Eigen::VectorXd res;
    res.setZero(noOfElem);
    std::istringstream f(str);
    std::string s;
    int i;
    for (i = 0; i < noOfElem; i++)
    {
        if(!getline(f, s, delimiter))
        {
            std::cerr << "Parse VectorXd error" << std::endl;
            break;
        }
        res(i) = std::stod(s);
    }
    if(i != noOfElem)
    {
        std::cerr << "Parse VectorXd error" << std::endl;
        return Eigen::VectorXd::Zero(noOfElem);
    }
    return res;
}
