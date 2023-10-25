#pragma once
#include <stdint.h>
#include <functional>
#include "status.hpp"

//Based on https://bitbucket.org/arrizza-public/algorithms/src/master/

/// @brief Simulation of real-time synchronized loop
class TimedLoop
{
    public:
        /// @brief Constructor
        /// @param periodInMs loop period in milliseconds
        /// @param func function that should be called in loop
        /// @param status reference to controlling status
        TimedLoop(int periodInMs, std::function<void(void)> func,Status& status);
        /// @brief start infinite loop
        void go();
        /// @brief start loop for specific cycle numbers
        /// @param loops how many cycles should be done
        void go(uint32_t loops);

    private:
        /// @brief Get process time
        /// @return process time in nanoseconds
        uint64_t get_current_clock_ns();
        /// @brief Calculate time that should be wait
        /// @return time to wait in nanosecounds
        int64_t calc_time_to_wait();
        /// @brief When next loop should be called
        uint64_t next_clock;

        /// @brief loop function
        std::function<void(void)> on_tick;
        uint64_t period;
        Status& _status;

        static constexpr uint64_t one_ms_in_ns = 1000000L;
};

