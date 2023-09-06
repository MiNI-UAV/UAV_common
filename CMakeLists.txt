cmake_minimum_required(VERSION 3.5)

project(common)

# Create the library
add_library(${PROJECT_NAME} STATIC)

# Include directories for header files
target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/header
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/logger
        ${CMAKE_CURRENT_SOURCE_DIR}/src/PID
        ${CMAKE_CURRENT_SOURCE_DIR}/src/RK4
        ${CMAKE_CURRENT_SOURCE_DIR}/src/timed_loop
        ${CMAKE_CURRENT_SOURCE_DIR}/src/parser
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# Add source files
target_sources(${PROJECT_NAME}
    PRIVATE
        src/logger/logger.cpp
        src/PID/PID.cpp
        src/RK4/RK4.cpp
        src/timed_loop/timed_loop.cpp
        src/parser/parser.cpp
)
