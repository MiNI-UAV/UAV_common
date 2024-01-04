#include "PID_discrete.hpp"
#include <iostream>
#include <string>
#include <cstring>

controllers::PID_Discrete::PID_Discrete(double Kp, double Ki, double Kd,
                                        double Kff, double N, double min,
                                        double max) 
    :
    _max{max},
    _min{min},
    _Kp{Kp},
    _Ki{Ki},
    _Kd{Kd},
    _Kff{Kff},
    _N{N}                                 
{}

controllers::PID_Discrete::PID_Discrete(rapidxml::xml_node<> *controller_node)
{
    for (rapidxml::xml_node<>* node = controller_node->first_node(); node; node = node->next_sibling()) 
    {
        if(std::strcmp(node->name(),"P") == 0) _Kp = std::stod(node->value());
        if(std::strcmp(node->name(),"I") == 0) _Ki = std::stod(node->value());
        if(std::strcmp(node->name(),"D") == 0) _Kd = std::stod(node->value());
        if(std::strcmp(node->name(),"FF") == 0) _Kff = std::stod(node->value());
        if(std::strcmp(node->name(),"min") == 0) _min = std::stod(node->value());
        if(std::strcmp(node->name(),"max") == 0) _max = std::stod(node->value());
        if(std::strcmp(node->name(),"type") == 0) assert(std::strcmp(node->value(),"PID_DISCRETE") == 0);
    }
}

double controllers::PID_Discrete::calc(double desired, double actual,
                                       double dt) {
  if (_pid_discrete == nullptr) {
    std::cerr << "dt not set!" << std::endl;
    return 0.0;
  }
  return _pid_discrete->calc(desired, actual);
}

void controllers::PID_Discrete::set_dt(double dt)
{
    Controller::set_dt(dt);
    std::array<double,3> num {{
        _Kp * (1.0 + _N*dt) + _Ki*dt*(1.0 + _N*dt) + _Kd*_N + _Kff * (1.0 + _N*dt),
        -(_Kp * (2.0 + _N*dt) + _Ki*dt + 2*_Kd*_N) - _Kff *(2.0 + _N*dt),
        _Kp + _Kd*_N + _Kff
        }};
    std::array<double,3> den {{
        1.0 + _N*dt,
        -(2.0 + _N*dt),
        1.0
        }};
    
    _pid_discrete = std::make_unique<ZTransformStatic<3,3>>(num, den,_min,_max);
}

void controllers::PID_Discrete::clear() 
{
    if(_pid_discrete != nullptr)
    {
        _pid_discrete->clear();
    }
}

std::unique_ptr<Controller> controllers::PID_Discrete::clone() const
{
    auto copied = std::unique_ptr<Controller>(new PID_Discrete(_Kp,_Ki,_Kd, _Kff,_min,_max));
    copied->set_dt(_dt);
    copied->clear();
    return copied;
}
