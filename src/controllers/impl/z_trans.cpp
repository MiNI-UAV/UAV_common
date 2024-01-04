#include "z_trans.hpp"
#include <sstream>
#include <iterator>
#include <string>
#include <cstring>

std::vector<double> splitStringToDoubleVector(const std::string &input) {
    std::istringstream iss(input);
    return {std::istream_iterator<double>(iss), std::istream_iterator<double>()};
}


using namespace controllers;

ZTransform::ZTransform(const std::vector<double>& num, const std::vector<double>& den,
    double min, double max) 
    : _min{min}, _max{max}, _num{num}, _den{den}
{
    if(num.size() > den.size())
    {
        throw std::runtime_error("num degree can not be higher that den");
    }
    auto multiplier = den.front();
    if(multiplier == 0.0)
    {
        throw std::runtime_error("first elem of den can not be zero!");
    }
    std::for_each(_num.begin(), _num.end(),
                [multiplier](auto x) { return x / multiplier; });
    std::for_each(_den.begin(), _den.end(),
                [multiplier](auto x) { return x / multiplier; });
    clear();
}

controllers::ZTransform::ZTransform(rapidxml::xml_node<> *controller_node)
{
    for (rapidxml::xml_node<>* node = controller_node->first_node(); node; node = node->next_sibling()) 
    {
        if(std::strcmp(node->name(),"num") == 0) _num = splitStringToDoubleVector(node->value());
        if(std::strcmp(node->name(),"den") == 0) _den = splitStringToDoubleVector(node->value());
        if(std::strcmp(node->name(),"min") == 0) _min = std::stod(node->value());
        if(std::strcmp(node->name(),"max") == 0) _max = std::stod(node->value());
        if(std::strcmp(node->name(),"type") == 0) assert(std::strcmp(node->value(),"Z_TRANSFORM") == 0);
    }
}

double ZTransform::calc(double desired, double actual, [[maybe_unused]] double dt)
{
    for(int i = _num.size()-1; i >= 1; i--)
    {
        _num_history[i] = _num_history[i-1];
    }
    _num_history[0] = (desired - actual);

    double res = std::inner_product(_num_history.begin(), _num_history.end(), _num.begin(), 0.0);
    res -= std::inner_product(_den_history.begin(), _den_history.end(), std::next(_den.begin(),1), 0.0);

    for(int i = _den.size()-2; i >= 1; i--)
    {
        _den_history[i] = _den_history[i-1];
    }
    _den_history[0] = std::clamp(res,_min,_max);;

    return _den_history[0];
}

void ZTransform::clear()
{
    _num_history.resize(_num.size());
    std::fill(_num_history.begin(), _num_history.end(), 0.0);
    _den_history.resize(_den.size() - 1);
    std::fill(_den_history.begin(), _den_history.end(), 0.0);
}

std::unique_ptr<Controller> ZTransform::clone() const
{
    return std::make_unique<ZTransform>(_num, _den);
}
