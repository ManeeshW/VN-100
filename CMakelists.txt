cmake_minimum_required(VERSION 3.10)
project(VN100_IMU)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")

# Find VectorNav library
set(VECTORNAV_PATH "${PROJECT_SOURCE_DIR}/libraries/vectornav/")
if(EXISTS "${VECTORNAV_PATH}/CMakeLists.txt")
    include_directories(${VECTORNAV_PATH}/include)
    add_subdirectory(${VECTORNAV_PATH} libvncxx)
else()
    message(FATAL_ERROR "VectorNav library not found at ${VECTORNAV_PATH}. Please ensure it is available.")
endif()

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Check for source files
set(SOURCES
    src/vn100.cpp
    src/main.cpp
)
foreach(source ${SOURCES})
    if(NOT EXISTS "${PROJECT_SOURCE_DIR}/${source}")
        message(FATAL_ERROR "Source file ${source} not found.")
    endif()
endforeach()

# Create executable
add_executable(vn100_imu ${SOURCES})
target_link_libraries(vn100_imu
    libvncxx
)
target_compile_options(vn100_imu
    PUBLIC -Wall -O3 -std=c++17
)