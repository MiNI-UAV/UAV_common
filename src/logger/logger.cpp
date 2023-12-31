#include "logger.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <initializer_list>
#include <string>
#include <filesystem>
namespace fs = std::filesystem;

fs::path Logger::log_path = "logs";

bool shouldLog(uint8_t group)
{
    return (LOGGER_MASK & (1 << group)) != 0;
}

Logger::Logger(std::string path, std::string fmt, uint8_t group):
    group{group}
{
    if(!shouldLog(group)) return;
    file.open(log_path / path);
    if(fmt != "") file << fmt << std::endl;
}

Logger::~Logger() 
{
    if(!shouldLog(group)) return;
    file.close();
}

void Logger::setFmt(std::string fmt)
{
    file << fmt << std::endl;
}

void Logger::log(double time, std::initializer_list<Eigen::VectorXd> args)
{
    static Eigen::IOFormat commaFormat(3, Eigen::DontAlignCols," ",",");
    if(!shouldLog(group)) return;
    file << time;
    for(auto v: args)
    {
        file << ',' << v.format(commaFormat);
    }
    file << std::endl;
}

void Logger::log(double time, std::initializer_list<double> args)
{
    if(!shouldLog(group)) return;
    file << time;
    for(auto v: args)
    {
        file << ',' << v;
    }
    file << std::endl;
}

void Logger::setLogDirectory(std::string subdirectory) 
{
    std::string session;
    std::ifstream session_read(log_path / "session");
    session_read >> session;
    log_path /= session;
    fs::create_directories(log_path / subdirectory);
    log_path /= subdirectory;
}