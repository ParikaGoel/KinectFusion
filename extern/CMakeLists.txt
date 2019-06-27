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
#if(NOT EXISTS "${PROJECT_SOURCE_DIR}/extern/pybind11/CMakeLists.txt")
#    message(FATAL_ERROR "Problems with submodule pybind.")
#endif()
#if(NOT EXISTS "${PROJECT_SOURCE_DIR}/extern/spdlog/CMakeLists.txt")
#    message(FATAL_ERROR "Problems with submodule  spdlog.")
#endif()
#add_subdirectory(eigen-git-mirror EXCLUDE_FROM_ALL)
#
#add_subdirectory(eigen-git-mirror EXCLUDE_FROM_ALL)
#add_subdirectory(Catch2 EXCLUDE_FROM_ALL)
add_library(eigenI INTERFACE)
target_include_directories(eigenI INTERFACE ${PROJECT_SOURCE_DIR}/extern/eigen-git-mirror)
#target_sources(eigenI INTERFACE ${PROJECT_SOURCE_DIR}/extern/eigen-git-mirror/Eigen/src)
add_library(catch2I INTERFACE)
target_include_directories(catch2I INTERFACE ${PROJECT_SOURCE_DIR}/extern/Catch2/include)

#add_subdirectory(Catch2)
##add_subdirectory(pybind11)
#add_subdirectory(spdlog)


#//https://foonathan.net/blog/2016/07/07/cmake-