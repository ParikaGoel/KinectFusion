cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME KinectFusion)
project(${PROJECT_NAME})
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set (CMAKE_CXX_STANDARD 14)

find_package(FreeImage REQUIRED)

#adds gitsubmodules
add_subdirectory(extern)
### if CMAKE Build Time
#adds our KinectFusion Lib
add_subdirectory(FusionLib)

#<-------Stuff for directly building an Application----->
set(APP_ONE ${PROJECT_NAME} )
add_executable(${APP_ONE} main.cpp)
#target_link_libraries (${TEST_TARGET_ONE} graph Eigen3::Eigen )# libdl)# pybind11)
target_link_libraries (${APP_ONE} kfusion eigenI ${FREEIMAGE_LIBRARIES})# libdl)# pybind11)
target_compile_features(${APP_ONE} PUBLIC cxx_std_17)


### Copy Data to cmake-build, this need to be applied to each new executable###
add_custom_command(TARGET ${PROJECT_NAME} PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/data $<TARGET_FILE_DIR:${PROJECT_NAME}>/data)

