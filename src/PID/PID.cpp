#include "PID.hpp"
#include <limits>
#include <algorithm>

PID::PID(double Kp, double Ki, double Kd,
         double min, double max, AntiWindUpMode antiWindUp):
    _dt(1.0),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0),
    _antiWindUp(antiWindUp)
{     
}

PID::~PID()
{
}

void PID::set_dt(double dt) 
{
    _dt = dt;
}

double PID::calc(double error)
{
    return calc(error,_dt);
}

double PID::calc(double error, double dt)
{
    double Pout = _Kp * error;

    double dI = error * dt;
    _integral += dI;
    double Iout = _Ki * _integral;

    double derivative = (error - _pre_error) / dt;
    double Dout = _Kd * derivative;

    double output = Pout + Iout + Dout;

    output = std::clamp(output,_min,_max);

    //ANTI-WINDUP - CLAMPING
    if(_antiWindUp == AntiWindUpMode::Clamping)
    {
        if((error > 0 && output == _max) || (error < 0 && output == _min))
        _integral -= dI;
    }

    _pre_error = error;

    return output;
}

void PID::clear()
{
    _integral = 0;
}
