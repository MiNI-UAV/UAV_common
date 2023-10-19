#pragma once
#include <functional>
#include <memory>
#include <Eigen/Dense>

class ODE
{
public:
    virtual Eigen::VectorXd step(double t,
                          Eigen::VectorXd y0,
                          std::function<Eigen::VectorXd(double,Eigen::VectorXd)> rhs_fun,
                          double h) = 0;

    enum ODEMethod { 
        Euler,
        Heun,
        RK4,
        NONE
    };

    static ODEMethod fromString(std::string str);
    static std::unique_ptr<ODE> factory(ODEMethod method);
};
