#include "bang_bang.hpp"
#include <cstring>
#include <string>

controllers::BangBang::BangBang(double high, double low, double delta)
    : _high{high}, _low{low}, _delta{delta}
{}

controllers::BangBang::BangBang(rapidxml::xml_node<> *controller_node)
    : BangBang(0.0,0.0)
{
    for (rapidxml::xml_node<>* node = controller_node->first_node(); node; node = node->next_sibling()) 
    {
        if(std::strcmp(node->name(),"low") == 0) _low = std::stod(node->value());
        if(std::strcmp(node->name(),"high") == 0) _high = std::stod(node->value());
        if(std::strcmp(node->name(),"delta") == 0) _delta = std::stod(node->value());
        if(std::strcmp(node->name(),"type") == 0) assert(std::strcmp(node->value(),"BANGBANG") == 0);
    }
}

double controllers::BangBang::calc(double desired, double actual, [[maybe_unused]] double dt) 
{ 
    double error = desired - actual;

    if(_last_output_sign > 0)
    {
        if(error < -_delta)
        {
            _last_output_sign = -1;
            return _low;
        }
        return _high;
    }

    if(_last_output_sign < 0)
    {
        if(error > _delta)
        {
            _last_output_sign = 1;
            return _high;
        }
        return _low;
    }
        
    // No last sign case
    if(error >= 0.0) 
    {
        _last_output_sign = 1;
        return _high;
    }
    _last_output_sign = -1;
    return _low;
}

void controllers::BangBang::clear() 
{
    _last_output_sign = 0;
}

std::unique_ptr<Controller> controllers::BangBang::clone() const 
{
    auto copied = std::unique_ptr<Controller>(new BangBang(_high,_low,_delta));
    copied->set_dt(_dt);
    copied->clear();
    return copied;
}
