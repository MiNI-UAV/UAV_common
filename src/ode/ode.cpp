#include "ode.hpp"
#include "ode_impl.hpp"

ODE::ODE(int micro_steps)
    : _micro_steps{micro_steps}
{}

int ODE::getMicrosteps() const { return _micro_steps; }

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

int ODE::getMicrosteps(ODEMethod method)
{
    if(method == ODE::ODEMethod::NONE)
    {
        return 0;
    }
    auto ode = factory(method);
    return ode->getMicrosteps();
}
