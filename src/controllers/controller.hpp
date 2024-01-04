#pragma once

#include <memory>
#include "rapidxml/rapidxml.hpp"

class Controller
{
    public:
        /// @brief Default constructor
        Controller() 
            : _dt{1.0}
        {}

        /// @brief Empty deconstructor for derived classes
        ~Controller() {}

        /// @brief Set new time step
        /// @param dt new time step
        virtual void set_dt(double dt) { _dt = dt; }

        /// @brief calc output of controller
        /// @param desired input of controller, desired value
        /// @param actual measured actual value
        /// @return output of controller
        double calc(double desired, double actual) { return calc(desired, actual,_dt); };

        /// @brief calc output of controller with specific time step
        /// @param desired input of controller, desired value
        /// @param actual measured actual value
        /// @param dt time step
        /// @return output of controller
        virtual double calc(double desired, double actual, double dt)  = 0;

        /// @brief clear internal state
        virtual void clear()  = 0;

        /// @brief virtual clone method
        virtual std::unique_ptr<Controller> clone() const = 0;

        /// @brief construct controller from given node. If xml is not valid return nullptr.
        /// @param controller_node xml node with controller config
        static std::unique_ptr<Controller> ControllerFactory(rapidxml::xml_node<>* controller_node);

    protected:
        double _dt;
};