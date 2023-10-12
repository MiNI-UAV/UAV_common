cmake_minimum_required(VERSION 3.5)

project(common)

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

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
        ${CMAKE_CURRENT_SOURCE_DIR}/src/components
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# Add source files
target_sources(${PROJECT_NAME}
    PRIVATE
    ${SOURCE_DIR}/logger/logger.cpp
        ${SOURCE_DIR}/PID/PID.cpp
        ${SOURCE_DIR}/RK4/RK4.cpp
        ${SOURCE_DIR}/timed_loop/timed_loop.cpp
        ${SOURCE_DIR}/parser/parser.cpp
        ${SOURCE_DIR}/parser/uav_params.cpp
        ${SOURCE_DIR}/components/aero_cofficients.hpp
        ${SOURCE_DIR}/components/components.hpp
        ${SOURCE_DIR}/components/control_surfaces.cpp
        ${SOURCE_DIR}/components/control_surfaces.hpp
        ${SOURCE_DIR}/components/drive.cpp
        ${SOURCE_DIR}/components/drive.hpp
        ${SOURCE_DIR}/components/hinge.cpp
        ${SOURCE_DIR}/components/hinge.hpp
        ${SOURCE_DIR}/components/loads.hpp
        ${SOURCE_DIR}/components/loads.cpp
        ${SOURCE_DIR}/components/navi.hpp
)
