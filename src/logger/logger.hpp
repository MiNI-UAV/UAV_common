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

/// @brief Log vector data with timestamp in file
class Logger
{
public:
    /// @brief Constructor
    /// @param path relative path with log file name.
    /// @param fmt format - information about log structure. First line in log file
    /// @param group log group - log will be created only if group is in actual LOGGER_MASK
    Logger(std::string path, std::string fmt = "", uint8_t group = 0);
    /// @brief Deconstructor
    ~Logger();

    /// @brief Set new format if was not known in constructor
    /// @param fmt new format
    void setFmt(std::string fmt);
    /// @brief Log one row
    /// @param time timestamp
    /// @param args list of double vectors
    void log(double time, std::initializer_list<Eigen::VectorXd> args);
    /// @brief Log one row
    /// @param time timestamp
    /// @param args list of doubles
    void log(double time, std::initializer_list<double> args);

    /// @brief Set global path that log should be created at.
    /// Path will be added to relative path of specific log instance
    /// @param subdirectory new global log path
    static void setLogDirectory(std::string subdirectory);

private:
    std::ofstream file;
    const uint8_t group;
    static fs::path log_path;
};

