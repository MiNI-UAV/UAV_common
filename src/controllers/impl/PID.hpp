#pragma once

#include <memory>
#include <limits>
#include "rapidxml/rapidxml.hpp"
#include "../controller.hpp"

namespace controllers
{
    class PID : public Controller
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
            /// @param Kd D term
            /// @param min saturation - lower range limit
            /// @param max saturation - upper range limit
            /// @param antiWindUp antiwindup method
            PID(double Kp, double Ki, double Kd,
                double min = std::numeric_limits<double>::min(),
                double max = std::numeric_limits<double>::max(),
                AntiWindUpMode antiWindUp = AntiWindUpMode::CLAMPING);


            /// @brief Construct controller with parameters from xml
            /// @param controller_node xml node with controller params
            PID(rapidxml::xml_node<>* controller_node);

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
            double _max;
            double _min;
            double _Kp;
            double _Kd;
            double _Ki;
            double _pre_error;
            double _integral;
            AntiWindUpMode _antiWindUp;
    };
}