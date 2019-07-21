include_directories(include)
set(FUSION_Name kfusion)
###set Kinect Fusion Library sources###
set(FUSION_SOURCES
        src/icp.cpp
        src/VirtualSensor.cpp
        src/KinectSensor.cpp
        src/Frame.cpp
        src/Volume.cpp
        src/Raycast.cpp
        src/Fusion.cpp
        src/FreeImageHelper.cpp
        src/Marching_cubes.cpp)

add_library(${FUSION_Name}  ${FUSION_SOURCES})
target_include_directories(${FUSION_Name} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
### link external libraries to Kinect Fusion Library
target_link_libraries(${FUSION_Name}  eigen  librealsense ${FREEIMAGE_LIBRARIES})
set_target_properties(${FUSION_Name} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
