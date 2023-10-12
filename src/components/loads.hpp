#include <Eigen/Dense>
#include <atomic>

class Load
{
public:
    double getMass() { return _mass; }

    Eigen::Vector3d getOffset() { return _offset; }

    /// @brief Try to release load
    /// @param time 
    /// @return leftover ammount of loads. Return -1 if load is not ready and -2 if out of load
    int release(double time);

protected:
    Load() = default;
    Load(int ammount, double reload, Eigen::Vector3d offset, double mass);
    Load& operator=(const Load& other);

private:
    Eigen::Vector3d _offset;
    double _mass;
    double _reload;
    double _last_release;
    std::atomic_int _ammount;

};

class Ammo: public Load
{
public:
    Ammo() = default;
    Ammo(int ammount,
         double reload,
         Eigen::Vector3d offset,
         double mass,
         Eigen::Vector3d V0
         );
    
    Eigen::Vector3d getV0() { return _V0; }

protected:
    Eigen::Vector3d _V0;
};

class Cargo: public Load
{
public:
    Cargo() = default;
    Cargo(int ammount,
         double reload,
         Eigen::Vector3d offset,
         double mass);
};

