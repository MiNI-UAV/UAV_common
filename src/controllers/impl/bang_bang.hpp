#pragma once

#include <memory>
#include "rapidxml/rapidxml.hpp"
#include "../controller.hpp"

namespace controllers
{
    class BangBang : public Controller
    {
        public:

            /// @brief Constructor with all Bang-bang controller parameters
            /// @param high output when error is positive
            /// @param low output when error is negative
            /// @param delta histeresis symetrical to zero
            BangBang(double high, double low, double delta = 0.0);

            /// @brief Construct controller with parameters from xml
            /// @param controller_node xml node with controller params
            BangBang(rapidxml::xml_node<>* controller_node);

            /// @brief calc output of controller with specific time step
            /// @param error input of controller
            /// @param dt time step, unused
            /// @return output of controller
            double calc(double error, [[maybe_unused]] double dt) override;

            /// @brief clear internal state
            void clear() override;

            /// @brief virtual clone method
            std::unique_ptr<Controller> clone() const override;

        private:
            double _high;
            double _low;
            double _delta;
            int _last_output_sign;

    };
}