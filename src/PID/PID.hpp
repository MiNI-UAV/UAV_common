#pragma once
#include <limits>

/// @brief Methods of handling windup in controller
enum AntiWindUpMode
{
    None,Clamping
};

//Inspiration : https://gist.github.com/bradley219/5373998
/// @brief PID discrete controller
class PID
{
    public:
        PID(double Kp, double Ki, double Kd,
         double min = std::numeric_limits<double>::min(),
         double max = std::numeric_limits<double>::max(),
         AntiWindUpMode antiWindUp = AntiWindUpMode::Clamping);
        ~PID();
        /// @brief Set new time step
        /// @param dt new time step
        void set_dt(double dt);
        /// @brief calc output of controller
        /// @param error input of controller
        /// @return output of controller
        double calc(double error);
        /// @brief calc output of controller with specific time step
        /// @param error input of controller
        /// @param dt time step
        /// @return output of controller
        double calc(double error, double dt);
        /// @brief clear internal state
        void clear();


    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
        AntiWindUpMode _antiWindUp;
};