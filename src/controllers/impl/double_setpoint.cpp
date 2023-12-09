#include "double_setpoint.hpp"
#include <cstring>
#include <string>

controllers::DoubleSetpoint::DoubleSetpoint(double high, double mid, double low,
                                            double mid_range, double delta) 
    : _high{high}, _mid{mid}, _low{low}, _mid_range{mid_range}, _delta{delta} 
{}

controllers::DoubleSetpoint::DoubleSetpoint(
    rapidxml::xml_node<> *controller_node)
    : DoubleSetpoint(0.0,0.0,0.0,0.0)
{
    for (rapidxml::xml_node<>* node = controller_node->first_node(); node; node = node->next_sibling()) 
    {
        if(std::strcmp(node->name(),"low") == 0) _low = std::stod(node->value());
        if(std::strcmp(node->name(),"mid") == 0) _mid = std::stod(node->value());
        if(std::strcmp(node->name(),"high") == 0) _high = std::stod(node->value());
        if(std::strcmp(node->name(),"midRange") == 0) _mid_range = std::stod(node->value());
        if(std::strcmp(node->name(),"delta") == 0) _delta = std::stod(node->value());
        if(std::strcmp(node->name(),"type") == 0) assert(std::strcmp(node->value(),"DOUBLE_SETPOINT") == 0);
    }
}

double controllers::DoubleSetpoint::calc(double desired, double actual, double dt)
{
    double error = desired - actual;
    
    if(error > _mid_range + _delta)
    {
        _last_output_sign = 1;
        return _high;
    }

    if(error < -_mid_range - _delta)
    {
        _last_output_sign = -1;
        return _low;
    }

    if(_last_output_sign > 0)
    {
        if(error < _mid_range - _delta)
        {
            _last_output_sign = 0;
            return _mid;
        }
        return _high;
    }

    if(_last_output_sign < 0)
    {
        if(error > -_mid_range + _delta)
        {
            _last_output_sign = 0;
            return _mid;
        }
        return _low;
    }  

    return _mid;
}

void controllers::DoubleSetpoint::clear() 
{
    _last_output_sign = 0;
}

std::unique_ptr<Controller> controllers::DoubleSetpoint::clone() const
{
    auto copied = std::unique_ptr<Controller>(new DoubleSetpoint(_high,_mid,_low,_mid_range,_delta));
    copied->set_dt(_dt);
    copied->clear();
    return copied;
}
