#include "loads.hpp"
#include <limits>

Load::Load(int ammount, double reload, Eigen::Vector3d offset, double mass)
    : _ammount{ammount}, _reload{reload}, _offset{offset}, _mass{mass} 
{
     _last_release = std::numeric_limits<double>::min();
}

Load& Load::operator=(const Load &other) 
{
    _ammount.store(other._ammount.load());
    _reload = other._reload;
    _offset = other._offset;
    _mass = other._mass;
    _last_release = std::numeric_limits<double>::min();
    return *this;
}

int Load::release(double time) 
{
    if (_ammount.load() <= 0)
        return -2;

    if ((time - _last_release) > _reload) 
    {
        _last_release = time;
        return --_ammount;
    } 
    else 
    {
        return -1;
    }
}

Ammo::Ammo(int ammount, double reload, Eigen::Vector3d offset, double mass,
           Eigen::Vector3d V0)
    : Load(ammount,reload,offset,mass), _V0{V0} 
{}

Cargo::Cargo(int ammount, double reload, Eigen::Vector3d offset, double mass) 
    : Load(ammount,reload,offset,mass)        
{}
