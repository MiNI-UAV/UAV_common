#pragma once
#include <Eigen/Dense>
#include "hinge.hpp"

/// @brief Drive propelling aircraft
struct Drive
{
    Eigen::Vector3d position;
    Eigen::Vector3d axis;
    int noOfHinges;
    Hinge hinges[2];
};

struct Rotor : Drive
{
    double forceCoff;
    double torqueCoff;
    int direction;
    double timeConstant;
    double maxSpeed;
    double hoverSpeed;
};

class Jet : public Drive
{
public:
    int phases;
    Eigen::VectorXd thrust;
    Eigen::VectorXd time;
    

    /// @brief start jet engine
    /// @param time timestamp of start
    /// @return true if start succesful, false if already started
    bool start(double time);
    /// @brief get thrust in specific time 
    /// @param time timestamp
    /// @return thrust value in Newtons
    double getThrust(double time);
    /// @brief get last calculated thrust
    /// @return last calculated thrust
    double getLastThrust() {return lastThrust;};
    
private:
    /// @brief State of jet engine
    enum JetState
    {
        /// @brief Idle before start
        READY,
        /// @brief jet engine is running
        WORKING,
        /// @brief  jet engine was started and burnt
        BURNT
    } state = JetState::READY; 
    int currentPhase = 0;
    double startTime = -1.0; 
    double lastThrust = 0.0;
};