#pragma once

#include <memory>
#include <limits>
#include <array>
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include "rapidxml/rapidxml.hpp"
#include "../controller.hpp"


namespace controllers
{
    template<unsigned int N, unsigned int D>
    class ZTransform : public Controller
    {
        public:
            /// @brief Constructorof Z-Transform controller
            /// @param min saturation - lower range limit
            /// @param max saturation - upper range limit
            ZTransform(
                const std::array<double,N>& num,
                const std::array<double,D>& den,
                double min = -std::numeric_limits<double>::max(),
                double max = std::numeric_limits<double>::max()
                );


            /// @brief Construct controller with parameters from xml
            /// @param controller_node xml node with controller params
            ZTransform(rapidxml::xml_node<>* controller_node) = delete;

            /// @brief calc output of controller
            /// @param desired input of controller, desired value
            /// @param actual measured actual value
            /// @return output of controller
            double calc(double desired, double actual, [[maybe_unused]] double dt) override;

            /// @brief clear internal state
            void clear() override;

            /// @brief virtual clone method
            std::unique_ptr<Controller> clone() const override;

        private:
            std::array<double,N> _num;
            std::array<double,D> _den;
            std::array<double,N> _num_history;
            std::array<double,D-1> _den_history;
            double _max;
            double _min;
    };

     template<unsigned int N, unsigned int D>
    ZTransform<N, D>::ZTransform(const std::array<double, N>& num, const std::array<double, D>& den,
        double min, double max) 
        : _min{min}, _max{max}
    {
        static_assert(N <= D, "num degree can not be higher that den");
        auto multiplier = den.front();
        if(multiplier == 0.0)
        {
            throw std::runtime_error("first elem of den can not be zero!");
        }
        std::transform(num.begin(), num.end(),
                   _num.begin(),
                   [multiplier](auto x) { return x / multiplier; });
        std::transform(den.begin(), den.end(),
                   _den.begin(),
                   [multiplier](auto x) { return x / multiplier; });
        clear();
    }

    template<unsigned int N, unsigned int D>
    double ZTransform<N, D>::calc(double desired, double actual, [[maybe_unused]] double dt)
    {
        for(int i = N-1; i >= 1; i--)
        {
            _num_history[i] = _num_history[i-1];
        }
        _num_history[0] = (desired - actual);

        double res = std::inner_product(_num_history.begin(), _num_history.end(), _num.begin(), 0.0);
        res -= std::inner_product(_den_history.begin(), _den_history.end(), std::next(_den.begin(),1), 0.0);

        for(int i = D-2; i >= 1; i--)
        {
            _den_history[i] = _den_history[i-1];
        }
        _den_history[0] = std::clamp(res,_min,_max);;

        return _den_history[0];
    }

    template<unsigned int N, unsigned int D>
    void ZTransform<N, D>::clear()
    {
        for(auto& elem : _num_history)
        {
            elem = 0.0;
        }
        for(auto& elem : _den_history)
        {
            elem = 0.0;
        }
    }

    template<unsigned int N, unsigned int D>
    std::unique_ptr<Controller> ZTransform<N, D>::clone() const
    {
        return std::make_unique<ZTransform<N, D>>(_num, _den);
    }

    class ZTransformFactory
    {
    public:
        static std::unique_ptr<Controller> factory(rapidxml::xml_node<>* controller_node);
    };
}