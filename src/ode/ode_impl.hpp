#pragma once

#include "ode.hpp"

/// @brief Explicit Euler algorithm
class ODE_Euler: public ODE
{
public:
    ODE_Euler()
        : ODE(1)
    {}

    Eigen::VectorXd step(double t,
                          Eigen::VectorXd y0,
                          std::function<Eigen::VectorXd(double,Eigen::VectorXd)> rhs_fun,
                          double h) override
    {
        return y0 + h*rhs_fun(t, y0);
    }
};

/// @brief Second order explicit Heun algorithm
class ODE_Heun: public ODE
{
public:
    ODE_Heun()
        : ODE(2)
    {}

    Eigen::VectorXd step(double t,
                          Eigen::VectorXd y0,
                          std::function<Eigen::VectorXd(double,Eigen::VectorXd)> rhs_fun,
                          double h) override
    {
        auto k1 = rhs_fun(t    , y0);
        auto k2 = rhs_fun(t + h, y0 + h*k1);
        return y0 + (h/2.0)*(k1 + k2);
    }
};

/// @brief Fourth order Runge Kutta algorithm
class ODE_RK4: public ODE
{
public:
    ODE_RK4()
        : ODE(4)
    {}


    Eigen::VectorXd step(double t,
                          Eigen::VectorXd y0,
                          std::function<Eigen::VectorXd(double,Eigen::VectorXd)> rhs_fun,
                          double h) override
    {
        auto k1 = rhs_fun(t      , y0);
        auto k2 = rhs_fun(t + h/2, y0+ (h/2)*k1);
        auto k3 = rhs_fun(t + h/2, y0+ (h/2)*k2);
        auto k4 = rhs_fun(t + h  , y0+ h*k3);
        return y0 + (h/6.0)*(k1 + 2*k2 + 2*k3 + k4);
    }
};