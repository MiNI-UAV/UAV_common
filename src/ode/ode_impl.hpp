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

/// @brief Second order predictor-corrector method
/// Second order Adams-bashforth and Adams-moulton
class ODE_PC2: public ODE
{
public:
    ODE_PC2()
        : ODE(2), f_prev{Eigen::VectorXd::Zero(0)}
    {}

    Eigen::VectorXd step(double t,
                          Eigen::VectorXd y0,
                          std::function<Eigen::VectorXd(double,Eigen::VectorXd)> rhs_fun,
                          double h) override
    {
        if(f_prev.size() == 0)
        {
            f_prev = rhs_fun(t,y0);
            ODE_RK4 rk4;
            return rk4.step(t,y0,rhs_fun,h);
        }
        Eigen::VectorXd f = rhs_fun(t,y0);
        Eigen::VectorXd P = y0 + 1.5 * h * f - 0.5 * h * f_prev;
        Eigen::VectorXd res = y0 + 0.5 * h * (rhs_fun(t + h, P) + f);
        f_prev = f;
        return res;
    }
private:
    Eigen::VectorXd f_prev;
};

/// @brief Fourth order predictor-corrector method
/// Fourth order Adams-bashforth and Adams-moulton
class ODE_PC4: public ODE
{
public:
    ODE_PC4()
        : ODE(2), f_prev{Eigen::VectorXd::Zero(0),Eigen::VectorXd::Zero(0),Eigen::VectorXd::Zero(0)}, index{-3}
    {}

    Eigen::VectorXd step(double t,
                          Eigen::VectorXd y0,
                          std::function<Eigen::VectorXd(double,Eigen::VectorXd)> rhs_fun,
                          double h) override
    {
        if(index < 0)
        {
            ODE_RK4 rk4;
            f_prev[index+3] = rhs_fun(t,y0);
            index++;
            return rk4.step(t,y0,rhs_fun,h);
        }
        Eigen::VectorXd f = rhs_fun(t,y0);
        Eigen::VectorXd P = y0 + 55.0 / 24.0 * h * f 
            - 59.0 / 24.0 * h * f_prev[(index+2) % 3] 
            + 37.0 / 24.0 * h * f_prev[(index+1) % 3] 
            - 9.0 / 24.0 * h * f_prev[index];
        Eigen::VectorXd res = y0 + 9.0 / 24.0 * h * rhs_fun(t + h, P) 
            + 19.0 / 24.0 * h * f 
            - 5.0 / 24.0 * h * f_prev[(index+2) % 3] 
            + 1.0 / 24.0 * h * f_prev[(index+1) % 3];
        f_prev[index] = f;
        index = (index + 1) % 3;
        return res;
    }
private:
    int index;
    Eigen::VectorXd f_prev[3];
};