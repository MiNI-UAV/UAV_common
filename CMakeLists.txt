cmake_minimum_required(VERSION 3.5)

project(common)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
set(CMAKE_CXX_STANDARD 20)

# Include directories for header files
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/header
    ${CMAKE_CURRENT_SOURCE_DIR}/src/logger
    ${CMAKE_CURRENT_SOURCE_DIR}/src/PID
    ${CMAKE_CURRENT_SOURCE_DIR}/src/timed_loop
)

# Add source files
set(SOURCES
    src/logger/logger.cpp
    src/PID/PID.cpp
    src/timed_loop/timed_loop.cpp
)

# Create the library
add_library(${PROJECT_NAME} STATIC ${SOURCES})

include_directories(${INCLUDE_DIR})
link_directories(${LIB_DIR})

link_directories("/usr/lib/x86_64-linux-gnu")
link_directories("/usr/local/lib")
link_directories("/usr/local/include")

# Find Eigen3 package
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Link against Eigen3
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# Set the output directories for libraries and binaries
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)