cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME KinectFusion)
project(${PROJECT_NAME})

set (CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
add_definitions(-DPROJECT_DIR="${PROJECT_SOURCE_DIR}")

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

# for some reason clang doesn't include them in Wall whereas gcc does, so include them explicitly.
SET(EXTRA_WARNING_FLAGS "-Wsign-compare")
SET(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG -ftree-vectorize")
SET(CMAKE_CXX_FLAGS " -ftemplate-backtrace-limit=0 -Wall ${EXTRA_WARNING_FLAGS} ${CMAKE_CXX_FLAGS}")

find_package(FreeImage REQUIRED)

find_package(realsense2 REQUIRED)
include_directories(${realsense2_INCLUDE_DIRS})

#adds gitsubmodules
add_subdirectory(extern)
### if CMAKE Build Time
#adds our KinectFusion Lib
add_subdirectory(FusionLib)

#<-------Stuff for directly building an Application----->
set(APP_ONE ${PROJECT_NAME} )
add_executable(${APP_ONE} main.cpp)
#target_link_libraries
target_link_libraries (${APP_ONE} kfusion eigen ${FREEIMAGE_LIBRARIES} realsense2)
target_compile_features(${APP_ONE} PUBLIC cxx_std_17 )
target_compile_definitions(${APP_ONE} PRIVATE PROJECT_DATA_DIR="${CMAKE_SOURCE_DIR}/data")



### Copy Data to cmake-build, this need to be applied to each new executable###
add_custom_command(TARGET ${PROJECT_NAME} PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/data $<TARGET_FILE_DIR:${PROJECT_NAME}>/data)
