#include "ode.hpp"
#include "ode_impl.hpp"

ODE::ODEMethod ODE::fromString(std::string str)
{
    if(str.compare("Euler") == 0)
    {
        return ODE::ODEMethod::Euler;
    }
    if(str.compare("Heun") == 0)
    {
        return ODE::ODEMethod::Heun;
    }
    if(str.compare("RK4") == 0)
    {
        return ODE::ODEMethod::RK4;
    }
    return ODE::ODEMethod::NONE;
}

std::unique_ptr<ODE> ODE::factory(ODEMethod method)
{ 
    switch (method)
    {
    case ODE::ODEMethod::Euler:
        return std::make_unique<ODE_Euler>();
    
    case ODE::ODEMethod::Heun:
        return std::make_unique<ODE_Heun>();
    
    case ODE::ODEMethod::RK4:
        return std::make_unique<ODE_RK4>();
    
    default:
        return std::unique_ptr<ODE>(); 
    }
}
