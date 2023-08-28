#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <initializer_list>
#include <string>
#include <filesystem>
namespace fs = std::filesystem;

#ifndef LOGGER_MASK
#define LOGGER_MASK -1
#endif

class Logger
{
public:
    Logger(std::string path, std::string fmt = "", uint8_t group = 0);
    ~Logger();

    void setFmt(std::string fmt);
    void log(double time, std::initializer_list<Eigen::VectorXd> args);
    void log(double time, std::initializer_list<double> args);

    static void setLogDirectory(std::string subdirectory);

private:
    std::ofstream file;
    const uint8_t group;
    static fs::path log_path;
};

