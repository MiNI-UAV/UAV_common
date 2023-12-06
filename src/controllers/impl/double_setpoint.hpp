#pragma once

#include <memory>
#include "rapidxml/rapidxml.hpp"
#include "../controller.hpp"

namespace controllers
{
    class DoubleSetpoint : public Controller
    {
        public:

            /// @brief Constructor with all Bang-bang controller parameters
            /// @param high output when error is in positive range
            /// @param mid output when error is in center range
            /// @param low output when error is in negative range
            /// @param mid_range size of center field from zero
            /// @param delta histeresis symetrical to zero
            DoubleSetpoint(double high, double mid, double low, double mid_range, double delta = 0.0);
        
            /// @brief Construct controller with parameters from xml
            /// @param controller_node xml node with controller params
            DoubleSetpoint(rapidxml::xml_node<>* controller_node);

            /// @brief calc output of controller with specific time step
            /// @param error input of controller
            /// @param dt time step
            /// @return output of controller
            double calc(double error, double dt) override;

            /// @brief clear internal state
            void clear() override;

            /// @brief virtual clone method
            std::unique_ptr<Controller> clone() const override;

        private:
            double _high;
            double _mid;
            double _low;
            double _mid_range;
            double _delta;
            int _last_output_sign;

    };
}