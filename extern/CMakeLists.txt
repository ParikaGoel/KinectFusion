find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    # Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()

if(NOT EXISTS "${PROJECT_SOURCE_DIR}/extern/eigen-git-mirror/CMakeLists.txt")
    message(FATAL_ERROR "Problems with submodule Eigen3.")
endif()
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/extern/Catch2/CMakeLists.txt")
    message(FATAL_ERROR "Problems with submodule CATCH2.")
endif()
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/extern/librealsense/CMakeLists.txt")
    message(FATAL_ERROR "Problems with submodule realsense.")
endif()

add_library(eigen INTERFACE)
target_include_directories(eigen INTERFACE ${PROJECT_SOURCE_DIR}/extern/eigen-git-mirror)
#target_sources(eigenI INTERFACE ${PROJECT_SOURCE_DIR}/extern/eigen-git-mirror/Eigen/src)
add_library(catch2 INTERFACE)
target_include_directories(catch2 INTERFACE ${PROJECT_SOURCE_DIR}/extern/Catch2/single_include)

add_library(librealsense INTERFACE)
target_include_directories(librealsense INTERFACE ${PROJECT_SOURCE_DIR}/extern/librealsense/include)

