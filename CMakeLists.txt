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

find_package(FreeImage REQUIRED)
include_directories(${FREEIMAGE_INCLUDE_DIRS})

set( HEADER_FILES
    ${PROJECT_SOURCE_DIR}/src/FreeImageHelper.h
    ${PROJECT_SOURCE_DIR}/src/VirtualSensor.h
)

set( SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/FreeImageHelper.cpp
    ${PROJECT_SOURCE_DIR}/src/DepthMapConverter.cpp
    ${PROJECT_SOURCE_DIR}/src/main.cpp
)

ADD_EXECUTABLE(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${FREEIMAGE_LIBRARIES})
