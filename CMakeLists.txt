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
        ${CMAKE_CURRENT_SOURCE_DIR}/src/ode
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
    ${SOURCE_DIR}/ode/ode_impl.hpp
    ${SOURCE_DIR}/ode/ode.cpp
    ${SOURCE_DIR}/ode/ode.hpp
    ${SOURCE_DIR}/timed_loop/timed_loop.cpp
    ${SOURCE_DIR}/parser/parser.cpp
    ${SOURCE_DIR}/parser/uav_params.cpp
    ${SOURCE_DIR}/components/aero_coefficients.hpp
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
    ${SOURCE_DIR}/controllers/controller.cpp
    ${SOURCE_DIR}/controllers/impl/PID.cpp
    ${SOURCE_DIR}/controllers/impl/PID_discrete.cpp
    ${SOURCE_DIR}/controllers/impl/bang_bang.cpp
    ${SOURCE_DIR}/controllers/impl/double_setpoint.cpp
    ${SOURCE_DIR}/controllers/impl/z_trans.cpp
)

# Tests
enable_testing() # Enable testing for the project

# Test target for ODE
add_executable(ODE_test ${SOURCE_DIR}/ode/ode_test.cpp ${SOURCE_FILES})
target_link_libraries(ODE_test ${PROJECT_NAME} gtest gtest_main)
add_test(NAME ODE_test COMMAND ODE_test)

# Test target for Controllers
add_executable(Controller_test ${SOURCE_DIR}/controllers/controller_test.cpp ${SOURCE_FILES})
target_link_libraries(Controller_test ${PROJECT_NAME} gtest gtest_main)
add_test(NAME Controller_test COMMAND Controller_test)
