

include_directories(include)
set(FUSION_Name kfusion)
###set Kinect Fusion Library sources###
set(FUSION_SOURCES
        src/icp.cpp
        src/VirtualSensor.cpp
        src/DepthSensor.cpp
        src/Frame.cpp
        src/Volume.cpp
        src/Raycast.cpp
        src/Fusion.cpp
        src/FreeImageHelper.cpp)

add_library(${FUSION_Name}  ${FUSION_SOURCES})
target_include_directories(${FUSION_Name} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
### link external libraries to Kinect Fusion Library
target_link_libraries(${FUSION_Name}  eigen  librealsense sophus  freeimage ${FREEIMAGE_LIBRARIES} ${CERES_LIBRARIES})
set_target_properties(${FUSION_Name} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

### Adds catch2 Tests in FusionLib/tests ###
MACRO (FUSIONLib_TEST NAME)
    SET(MODULES "${ARGN}")
    ADD_EXECUTABLE(test_${NAME} test_${NAME}.cpp ${PROJECT_SOURCE_DIR}/FusionLib/test_main.cpp)
    TARGET_LINK_LIBRARIES(test_${NAME} ${MODULES} eigen  catch2 freeimage )
    ADD_TEST(test_${NAME} test_${NAME})
ENDMACRO (FUSIONLib_TEST)
add_subdirectory(tests)
