#pragma once

#include <memory>
#include <limits>
#include "rapidxml/rapidxml.hpp"
#include "../controller.hpp"
#include "z_trans.hpp"

namespace controllers
{
    class PID_Discrete : public Controller
    {
        public:
        

            /// @brief Constructor with all PID controller parameters
            /// @param Kp P term
            /// @param Ki I term
            /// @param Kd D term
            /// @param Kff FF term
            /// @param min saturation - lower range limit
            /// @param max saturation - upper range limit
            /// @param antiWindUp antiwindup method
            PID_Discrete(double Kp, double Ki, double Kd, double Kff = 0.0,
                double N = 100.0,
                double min = -std::numeric_limits<double>::max(),
                double max = std::numeric_limits<double>::max());


            /// @brief Construct controller with parameters from xml
            /// @param controller_node xml node with controller params
            PID_Discrete(rapidxml::xml_node<>* controller_node);

            /// @brief calc output of controller with specific time step
            /// @param desired input of controller, desired value
            /// @param actual measured actual value
            /// @param dt time step
            /// @return output of controller
            double calc(double desired, double actual, double dt) override;

            void set_dt(double dt) override;

            /// @brief clear internal state
            void clear() override;

            /// @brief virtual clone method
            std::unique_ptr<Controller> clone() const override;

        private:
            double _max;
            double _min;
            double _Kp;
            double _Ki;
            double _Kd;
            double _Kff;
            double _N;

            std::unique_ptr<Controller> _pid_discrete;
    };
}