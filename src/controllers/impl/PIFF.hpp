#pragma once

#include <memory>
#include <limits>
#include "rapidxml/rapidxml.hpp"
#include "../controller.hpp"

namespace controllers
{
    class PIFF : public Controller
    {
        public:
        
            /// @brief Methods of handling windup in controller
            enum class AntiWindUpMode
            {
                NONE, CLAMPING
            };

            /// @brief Constructor with all PID controller parameters
            /// @param Kp P term
            /// @param Ki I term
            /// @param Kff FF term
            /// @param min saturation - lower range limit
            /// @param max saturation - upper range limit
            /// @param antiWindUp antiwindup method
            PIFF(double Kp, double Ki, double Kff,
                double min = -std::numeric_limits<double>::max(),
                double max = std::numeric_limits<double>::max(),
                AntiWindUpMode antiWindUp = AntiWindUpMode::CLAMPING);


            /// @brief Construct controller with parameters from xml
            /// @param controller_node xml node with controller params
            PIFF(rapidxml::xml_node<>* controller_node);

            /// @brief calc output of controller with specific time step
            /// @param desired input of controller, desired value
            /// @param actual measured actual value
            /// @param dt time step
            /// @return output of controller
            double calc(double desired, double actual, double dt) override;

            /// @brief clear internal state
            void clear() override;

            /// @brief virtual clone method
            std::unique_ptr<Controller> clone() const override;

        private:
            double _max;
            double _min;
            double _Kp;
            double _Ki;
            double _Kff;
            double _integral;
            AntiWindUpMode _antiWindUp;
    };
}