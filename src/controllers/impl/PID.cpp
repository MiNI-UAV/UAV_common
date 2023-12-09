#include "PID.hpp"
#include <algorithm>
#include <cstring>
#include <string>

using namespace controllers;

PID::PID(double Kp, double Ki, double Kd,
         double min, double max, AntiWindUpMode antiWindUp)
    :
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

PID::PID(rapidxml::xml_node<>* controller_node):
    PID(0.0,0.0,0.0)
{
    for (rapidxml::xml_node<>* node = controller_node->first_node(); node; node = node->next_sibling()) 
    {
        if(std::strcmp(node->name(),"P") == 0) _Kp = std::stod(node->value());
        if(std::strcmp(node->name(),"I") == 0) _Ki = std::stod(node->value());
        if(std::strcmp(node->name(),"D") == 0) _Kd = std::stod(node->value());
        if(std::strcmp(node->name(),"min") == 0) _min = std::stod(node->value());
        if(std::strcmp(node->name(),"max") == 0) _max = std::stod(node->value());
        if(std::strcmp(node->name(),"antyWindup") == 0)
        {
            if(std::strcmp(node->value(),"NONE") == 0) _antiWindUp = AntiWindUpMode::NONE;
            if(std::strcmp(node->name(),"CLAMPING") == 0) _antiWindUp = AntiWindUpMode::CLAMPING;
        }
        if(std::strcmp(node->name(),"type") == 0) assert(std::strcmp(node->value(),"PID") == 0);
    }
}

std::unique_ptr<Controller> PID::clone() const {
    auto copied = std::unique_ptr<Controller>(new PID(_Kp,_Ki,_Kd,_min,_max,_antiWindUp));
    copied->set_dt(_dt);
    copied->clear();
    return copied;
}

double PID::calc(double desired, double actual, double dt)
{
    double error = desired - actual;

    double Pout = _Kp * error;

    double dI = error * dt;
    _integral += dI;
    double Iout = _Ki * _integral;

    double derivative = (error - _pre_error) / dt;
    double Dout = _Kd * derivative;

    double output = Pout + Iout + Dout;

    output = std::clamp(output,_min,_max);

    //ANTI-WINDUP - CLAMPING
    if(_antiWindUp == AntiWindUpMode:: CLAMPING)
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

