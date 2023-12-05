#pragma once
#include <functional>
#include <memory>
#include <Eigen/Dense>

/// @brief Ordinal differencial equation solver
class ODE
{
public:
    /// @brief Constructor
    ODE(int micro_steps);

    /// @brief Virtual deconstructor
    virtual ~ODE() {}

    /// @brief One step of explicit solving algorithm
    /// @param t start time
    /// @param y0 start variable
    /// @param rhs_fun right-hand-side function, calculation of derivative
    /// @param h time step
    /// @return 
    virtual Eigen::VectorXd step(double t,
                          Eigen::VectorXd y0,
                          std::function<Eigen::VectorXd(double,Eigen::VectorXd)> rhs_fun,
                          double h) = 0;

    /// @brief Return microsteps - number of rhs function calls to calculate on step.
    /// @return microsteps
    int getMicrosteps() const;

    /// @brief Supported solving method
    enum ODEMethod { 
        Euler,
        Heun,
        RK4,
        PC2,
        PC4,
        NONE
    };

    /// @brief Parse solving method from string
    /// @param str input string
    /// @return solving method if parsed, NONE if unknown
    static ODEMethod fromString(std::string str);


    /// @brief Factory constructing ODE solvers
    /// @param method type of desired method
    /// @return instance of ODE solver
    static std::unique_ptr<ODE> factory(ODEMethod method);

    /// @brief Get microsteps of given method
    /// @param method method type
    /// @return number of microstep in one algoritm step
    static int getMicrosteps(ODEMethod method);

private:
    
    /// @brief define how many time rhs_fun will be called in one step
    const int _micro_steps;
};
