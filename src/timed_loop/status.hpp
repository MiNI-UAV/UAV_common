#pragma once


/// @brief status of timed loop. Control it's job
enum Status
{
    /// @brief loop is ready to run
    idle = 1, 
    /// @brief loop is running
    running = 2,
    /// @brief loop will be break in next occasion.
    exiting = 3,
    /// @brief loop job should be reloaded
    reload = 4
};