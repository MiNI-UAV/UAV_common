#include "controller.hpp"
#include "impl/PID.hpp"
#include <cstring>
#include <stdexcept>

std::unique_ptr<Controller>
Controller::ControllerFactory(rapidxml::xml_node<>* controller_node) 
{
   rapidxml::xml_node<>* type_node = nullptr;
   for (rapidxml::xml_node<>* node = controller_node->first_node(); node; node = node->next_sibling()) 
    {
        if(std::strcmp(node->name(),"type") == 0)
        {
            if(type_node != nullptr)
            {
                throw std::runtime_error("Multiple type in node!");
            }
            type_node = node;
        }
    }
    if(type_node == nullptr)
    {
        return std::unique_ptr<Controller>{};
    }

    if(std::strcmp(type_node->value(),"PID") == 0) return std::unique_ptr<Controller>(new controllers::PID(controller_node));

    return std::unique_ptr<Controller>{};
}