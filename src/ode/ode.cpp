#include "ode.hpp"

std::unique_ptr<ODE> ODE::factory(ODEMethod method) 
{ 
    return std::unique_ptr<ODE>(); 
}
