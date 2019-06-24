set(PROJECT_NAME KinectFusion)
project(${PROJECT_NAME})

cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

# Required libraries
find_package(Eigen3 REQUIRED)

INCLUDE_DIRECTORIES(
        ${EIGEN3_INCLUDE_DIR}
)

# build main executable
ADD_EXECUTABLE(${PROJECT_NAME}
        ${PROJECT_SOURCE_DIR}/src/main.cpp)
