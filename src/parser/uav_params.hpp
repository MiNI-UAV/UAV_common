#pragma once
#include <Eigen/Dense>
#include <mutex>
#include <memory>
#include <map>
#include "rapidxml/rapidxml.hpp"

#include "../components/components.hpp"
#include "../controllers/controller.hpp"


/// @brief Parsed UAV configuration from XML
struct UAVparams
{
    public:
        UAVparams();
        ~UAVparams();
        void loadConfig(std::string configFile);

        std::string name;

        bool instantRun;

        std::string initialMode;
        Eigen::Vector3d initialPosition;
        Eigen::Vector3d initialOrientation;
        Eigen::Vector3d initialVelocity;

        //Mass params
        double m;
        double Ix;
        double Iy;
        double Iz;
        double Ixy;
        double Ixz;
        double Iyz;

        //Rotor params
        int noOfRotors;
        std::unique_ptr<Rotor[]> rotors;
        Eigen::VectorXd getRotorTimeContants() const;
        Eigen::VectorXd getRotorMaxSpeeds() const;
        Eigen::VectorXd getRotorHoverSpeeds() const;

        //Jet params
        int noOfJets;
        std::unique_ptr<Jet[]> jets;

        //Surface params
        ControlSurfaces surfaces;

        //Aerodynamic params
        AeroCoefficients aero_coffs;

        std::map<std::string,std::unique_ptr<Controller>> controllers;

        std::vector<SensorParams> sensors;
        AHRSParams ahrs;
        EKFScalers ekf;

        Eigen::MatrixX4d rotorMixer;
        Eigen::MatrixX4d surfaceMixer;

        //Ammo params
        int noOfAmmo;
        std::unique_ptr<Ammo[]> ammo;

        //Cargo params
        int noOfCargo;
        std::unique_ptr<Cargo[]> cargo;

        const static UAVparams* getSingleton();

    private:
        
        void setMass(rapidxml::xml_node<> * interiaNode);
        void setInitial(rapidxml::xml_node<>* initialNode);
        void setRotors(rapidxml::xml_node<> * rotorsNode);
        void setJets(rapidxml::xml_node<> * rotorsNode);
        void setAero(rapidxml::xml_node<> * aeroNode);
        void setControlSurface(rapidxml::xml_node<> * surfaceNode);
        void setSensors(rapidxml::xml_node<>* sensorNode);
        void setAHRS(rapidxml::xml_node<>* AHRSNode);
        void setEKF(rapidxml::xml_node<>* EKFNode);
        void setMixers(rapidxml::xml_node<>* mixersNode);
        void setAmmo(rapidxml::xml_node<> * ammoNode);
        void setCargo(rapidxml::xml_node<> * cargoNode);

        static UAVparams* singleton;

};
