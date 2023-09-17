#pragma once
#include <Eigen/Dense>
#include <mutex>
#include <memory>
#include <map>
#include "rapidxml/rapidxml.hpp"

#include "../components/components.hpp"
#include "../PID/PID.hpp"


struct UAVparams
{
    public:
        UAVparams();
        ~UAVparams();
        void loadConfig(std::string configFile);

        std::string name;

        bool instantRun;

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
        AeroCofficients aero_coffs;

        std::map<std::string,PID> pids;
        std::function<Eigen::VectorXd(double,double,double,double)> mixer;

        std::vector<SensorParams> sensors;
        AHRSParams ahrs;
        EKFScalers ekf;

        Eigen::MatrixX4d rotorMixer;
        Eigen::MatrixX4d surfaceMixer;

        const static UAVparams* getSingleton();

    private:
        
        void setMass(rapidxml::xml_node<> * interiaNode);
        void setRotors(rapidxml::xml_node<> * rotorsNode);
        void setJets(rapidxml::xml_node<> * rotorsNode);
        void setAero(rapidxml::xml_node<> * aeroNode);
        void setControlSurface(rapidxml::xml_node<> * surfaceNode);
        void setSensors(rapidxml::xml_node<>* sensorNode);
        void setAHRS(rapidxml::xml_node<>* AHRSNode);
        void setEKF(rapidxml::xml_node<>* EKFNode);
        void setMixers(rapidxml::xml_node<>* mixersNode);

        static UAVparams* singleton;

};
