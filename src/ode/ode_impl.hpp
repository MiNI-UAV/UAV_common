#pragma once

#include "ode.hpp"

class ODE_Euler: public ODE
{
    Eigen::VectorXd step(double t,
                          Eigen::VectorXd y0,
                          std::function<Eigen::VectorXd(double,Eigen::VectorXd)> rhs_fun,
                          double h) override
    {
        return y0 + h*rhs_fun(t, y0);
    }
};

class ODE_Heun: public ODE
{
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

class ODE_RK4: public ODE
{
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