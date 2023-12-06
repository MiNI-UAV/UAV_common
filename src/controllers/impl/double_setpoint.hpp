#pragma once

#include <memory>
#include "rapidxml/rapidxml.hpp"
#include "../controller.hpp"

namespace controllers
{
    class DoubleSetpoint : public Controller
    {
        public:
        
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

    };
}