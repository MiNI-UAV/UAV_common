#include <Eigen/Dense>
#include "uav_params.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <mutex>
#include "rapidxml/rapidxml.hpp"
#include "parser.hpp"

/// @brief Initialize default data
UAVparams::UAVparams() 
{
    name = "default";
    
    if(singleton != nullptr)
    {
        std::cerr << "Only one instance of UAVParams should exist";
        return;
    }
    singleton = this;
}

UAVparams* UAVparams::singleton = nullptr;

Eigen::VectorXd UAVparams::getRotorTimeContants() const
{
    auto vec = Eigen::VectorXd(noOfRotors);
    for (int i = 0; i < noOfRotors; i++)
    {
        vec(i) = rotors[i].timeConstant;
    }
    return vec;
}

Eigen::VectorXd UAVparams::getRotorMaxSpeeds() const
{
    Eigen::VectorXd vec;
    vec.setZero(noOfRotors);
    for (int i = 0; i < noOfRotors; i++)
    {
        vec(i) = rotors[i].maxSpeed;
    }
    return vec;
}

const UAVparams *UAVparams::getSingleton()
{
    return singleton;
}

void UAVparams::setMass(rapidxml::xml_node<> *interiaNode)
{
    for (rapidxml::xml_node<>* node = interiaNode->first_node(); node; node = node->next_sibling()) 
    {
        if(std::strcmp(node->name(),"mass") == 0)
        {
            m = std::stod(node->value());
        }
        if(std::strcmp(node->name(),"Ix") == 0)
        {
            Ix = std::stod(node->value());
        }
        if(std::strcmp(node->name(),"Iy") == 0)
        {
            Iy = std::stod(node->value());
        }
        if(std::strcmp(node->name(),"Iz") == 0)
        {
            Iz = std::stod(node->value());
        }
        if(std::strcmp(node->name(),"Ixy") == 0)
        {
            Ixy = std::stod(node->value());
        }
        if(std::strcmp(node->name(),"Ixz") == 0)
        {
            Ixz = std::stod(node->value());
        }
        if(std::strcmp(node->name(),"Iyz") == 0)
        {
            Iyz = std::stod(node->value());
        }
    }
}

void parseHinge(rapidxml::xml_node<>* hingeNode, Hinge* hinge)
{
    Eigen::Vector3d axis(1.0,0.0,0.0);
    double max = 0.0;
    double min = 0.0;
    double trim = 0.0;
    for (rapidxml::xml_node<>* node = hingeNode->first_node(); node; node = node->next_sibling()) 
    {
        if(std::strcmp(node->name(),"axis") == 0)
        {
            double x,y,z;
            std::sscanf(node->value(),"%lf %lf %lf",&x,&y,&z);
            axis << x,y,z;
        }

        if(std::strcmp(node->name(),"min") == 0)
        {
            min = std::stod(node->value());
        }

        if(std::strcmp(node->name(),"max") == 0)
        {
            max = std::stod(node->value());
        }

        if(std::strcmp(node->name(),"trim") == 0)
        {
            trim = std::stod(node->value());
        }

    }
    *hinge = Hinge(axis,max,min,trim);
}

void UAVparams::setRotors(rapidxml::xml_node<> * rotorsNode)
{
    noOfRotors = std::stoi(rotorsNode->first_attribute()->value());

    if(noOfRotors == 0) return; 

    rotors = std::make_unique<Rotor[]>(noOfRotors);

    int i = 0;
    for (rapidxml::xml_node<>* rotorNode = rotorsNode->first_node(); rotorNode && i < noOfRotors; rotorNode = rotorNode->next_sibling(), i++) 
    {
        if(std::strcmp(rotorNode->name(),"rotor") != 0) continue;


        for(rapidxml::xml_node<>* node = rotorNode->first_node(); node; node = node->next_sibling())
        {


            if(std::strcmp(node->name(),"forceCoff") == 0)
            {
                rotors[i].forceCoff = std::stod(node->value());
            }

            if(std::strcmp(node->name(),"torqueCoff") == 0)
            {
                rotors[i].torqueCoff = std::stod(node->value());
            }

            if(std::strcmp(node->name(),"position") == 0)
            {
                double x,y,z;
                std::sscanf(node->value(),"%lf %lf %lf",&x,&y,&z);
                rotors[i].position << x,y,z;
            }

            if(std::strcmp(node->name(),"axis") == 0)
            {
                double x,y,z;
                std::sscanf(node->value(),"%lf %lf %lf",&x,&y,&z);
                rotors[i].axis << x,y,z;
            }

            if(std::strcmp(node->name(),"direction") == 0)
            {
                rotors[i].direction = std::stoi(node->value());
            }

            if(std::strcmp(node->name(),"timeConstant") == 0)
            {
                rotors[i].timeConstant = std::stod(node->value());
            }

            if(std::strcmp(node->name(),"maxSpeed") == 0)
            {
                rotors[i].maxSpeed = std::stod(node->value());
            }

            if(std::strcmp(node->name(),"hinges") == 0)
            {
                rotors[i].noOfHinges = std::stod(node->first_attribute()->value());
                int j = 0;
                for(rapidxml::xml_node<>* hingeNode = node->first_node(); hingeNode && j < rotors[i].noOfHinges; hingeNode = hingeNode->next_sibling(), j++)
                {
                    parseHinge(hingeNode, &(rotors[i].hinges[j]));
                }
            }
        }
    }
}

void UAVparams::setJets(rapidxml::xml_node<> * jetsNode)
{
    noOfJets = std::stoi(jetsNode->first_attribute()->value());

    if(noOfJets == 0) return; 

    jets = std::make_unique<Jet[]>(noOfJets);

    int i = 0;
    for (rapidxml::xml_node<>* jetNode = jetsNode->first_node(); jetNode && i < noOfJets; jetNode = jetNode->next_sibling(), i++) 
    {
        if(std::strcmp(jetNode->name(),"jet") != 0) continue;

        for(rapidxml::xml_node<>* node = jetNode->first_node(); node; node = node->next_sibling())
        {
            if(std::strcmp(node->name(),"position") == 0)
            {
                double x,y,z;
                std::sscanf(node->value(),"%lf %lf %lf",&x,&y,&z);
                jets[i].position << x,y,z;
            }

            if(std::strcmp(node->name(),"axis") == 0)
            {
                double x,y,z;
                std::sscanf(node->value(),"%lf %lf %lf",&x,&y,&z);
                jets[i].axis << x,y,z;
            }

            if(std::strcmp(node->name(),"phases") == 0)
            {
                jets[i].phases = std::stoi(node->value());
            }

            if(std::strcmp(node->name(),"thrust") == 0)
            {
                jets[i].thrust = parseVectorXd(node->value(),jets[i].phases);
            }

            if(std::strcmp(node->name(),"time") == 0)
            {
                jets[i].time = parseVectorXd(node->value(),jets[i].phases);
            }

            if(std::strcmp(node->name(),"hinges") == 0)
            {
                jets[i].noOfHinges = std::stod(node->first_attribute()->value());
                int j = 0;
                for(rapidxml::xml_node<>* hingeNode = node->first_node(); hingeNode && j < jets[i].noOfHinges; hingeNode = hingeNode->next_sibling(), j++)
                {
                    parseHinge(hingeNode, &(jets[i].hinges[j]));
                }
            }
        }
    }
}

void UAVparams::setAero(rapidxml::xml_node<> * aeroNode)
{
    for (rapidxml::xml_node<>* node = aeroNode->first_node(); node; node = node->next_sibling()) 
    {
        if(std::strcmp(node->name(),"S") == 0)
        {
            aero_coffs.S = std::stod(node->value());
        }
        if(std::strcmp(node->name(),"d") == 0)
        {
            aero_coffs.d = std::stod(node->value());
        }
        if(std::strcmp(node->name(),"eAR") == 0)
        {
            aero_coffs.eAR = std::stod(node->value());
        }
        if(std::strcmp(node->name(),"C0") == 0)
        {
            aero_coffs.C0 =  parseVectorXd(node->value(), 6, ',');
        }
        if(std::strcmp(node->name(),"Cpqr") == 0)
        {
            aero_coffs.Cpqr =  parseMatrixXd(node->value(), 6,3, ',');
        }
        if(std::strcmp(node->name(),"Cab") == 0)
        {
            aero_coffs.Cab =  parseMatrixXd(node->value(), 6,4, ',');
        }
    }
}

void UAVparams::setControlSurface(rapidxml::xml_node<> *surfaceNode)
{
    int noOfSurface = std::stoi(surfaceNode->first_attribute()->value());

    if(noOfSurface == 0) return; 

    Eigen::Matrix<double,6,-1> matrix;
    Eigen::VectorXd min, max, trim;
    matrix.setZero(6,0);
    max.setZero(0);
    min.setZero(0);
    trim.setZero(0);

    for (rapidxml::xml_node<>* node = surfaceNode->first_node(); node; node = node->next_sibling()) 
    {
        if(std::strcmp(node->name(),"matrix") == 0)
        {
            matrix = parseMatrixXd(node->value(),6,noOfSurface,',');
        }
        if(std::strcmp(node->name(),"min") == 0)
        {
            min = parseVectorXd(node->value(),noOfSurface,',');
        }
        if(std::strcmp(node->name(),"max") == 0)
        {
            max = parseVectorXd(node->value(),noOfSurface,',');
        }
        if(std::strcmp(node->name(),"trim") == 0)
        {
            trim = parseVectorXd(node->value(),noOfSurface,',');
        }
    }


    surfaces = ControlSurfaces(noOfSurface,matrix,min,max,trim);
}

void UAVparams::loadConfig(std::string configFile)
{
    if(!std::filesystem::exists(configFile))
    {  
        throw std::runtime_error("Config file not exist!");
    }
    std::cout << "Loading config" << std::endl;
    std::ifstream file(configFile);
    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    rapidxml::xml_document<> doc;
    doc.parse<0>(&content[0]);
    rapidxml::xml_node<>* root = doc.first_node("params");
    for (rapidxml::xml_node<>* node = root->first_node(); node; node = node->next_sibling()) 
    {
        if(std::strcmp(node->name(),"name") == 0)
        {
            name.assign(node->value(), node->value_size());
        }
        if(std::strcmp(node->name(),"ineria") == 0)
        {
            setMass(node);
        }
        if(std::strcmp(node->name(),"rotors") == 0)
        {
            setRotors(node);
        }
        if(std::strcmp(node->name(),"jets") == 0)
        {
            setJets(node);
        }
        if(std::strcmp(node->name(),"aero") == 0)
        {
            setAero(node);
        }
        if(std::strcmp(node->name(),"surface") == 0)
        {
            setControlSurface(node);
        }
    }
}

UAVparams::~UAVparams()
{
    singleton = nullptr;
}