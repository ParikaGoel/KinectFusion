cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME KinectFusion)
project(${PROJECT_NAME})

set (CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

# for some reason clang doesn't include them in Wall whereas gcc does, so include them explicitly.
SET(EXTRA_WARNING_FLAGS "-Wsign-compare")
SET(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG -ftree-vectorize")
SET(CMAKE_CXX_FLAGS " -ftemplate-backtrace-limit=0 -Wall ${EXTRA_WARNING_FLAGS} ${CMAKE_CXX_FLAGS}")

find_package(Eigen3 3.3.4 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Ceres 1.14 REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${SOPHUS_INCLUDE_DIR})

find_package(FreeImage REQUIRED)

find_package(realsense2 REQUIRED)
include_directories(${realsense2_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)

# Source Files
set(SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/src/icp.cpp
        ${PROJECT_SOURCE_DIR}/src/FreeImageHelper.cpp
        ${PROJECT_SOURCE_DIR}/src/DepthSensor.cpp
        ${PROJECT_SOURCE_DIR}/src/Frame.cpp)

# Libraries to be linked
set(TARGET_LIBS
    ${CERES_LIBRARIES}
    ${FREEIMAGE_LIBRARIES}
    realsense2
    )

add_executable(${PROJECT_NAME} main.cpp ${SOURCE_FILES})
target_link_libraries (${PROJECT_NAME} ${TARGET_LIBS})
