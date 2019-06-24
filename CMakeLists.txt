set(PROJECT_NAME KinectFusion)
project(${PROJECT_NAME})

cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

# Required libraries
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${SOPHUS_INCLUDE_DIR})

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/src)

# Source Files
set(SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/src/icp/icp.cpp)

# Libraries to be linked
set(TARGET_LIBS
        ${CERES_LIBRARIES})

# build main executable
ADD_EXECUTABLE(${PROJECT_NAME}
        ${SOURCE_FILES}
        ${PROJECT_SOURCE_DIR}/src/main.cpp)

target_link_libraries(${PROJECT_NAME} ${TARGET_LIBS})