cmake_minimum_required(VERSION 3.10)
project(johannes_mpc)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(CASADI REQUIRED)
find_package(PythonLibs 2.7)

add_library(jintegrator ThirdParty/polympc/src/integrator.cpp ThirdParty/polympc/src/integrator.h)
target_link_libraries(jintegrator Eigen3::Eigen polymath)

add_library(visualization_tools visualization_tools.cpp visualization_tools.h)
target_link_libraries(visualization_tools ${CASADI_LIBRARIES})

add_library(jkite kite.cpp kite.h)
target_link_libraries(jkite ${CASADI_LIBRARIES})

add_library(matplotlib matplotlibcpp.cpp matplotlibcpp.h)
target_link_libraries(matplotlib ${PYTHON_LIBRARIES})

add_library(wheeldyn_bicycle wheeldyn_bicycle.cpp wheeldyn_bicycle.h)
target_link_libraries(wheeldyn_bicycle matplotlib ${CASADI_LIBRARIES})

add_library(simple_bicycle simple_bicycle.cpp simple_bicycle.hpp)
target_link_libraries(simple_bicycle matplotlib wheeldyn_bicycle ${CASADI_LIBRARIES})

add_library(kinematic_bicycle kinematic_bicycle.cpp kinematic_bicycle.hpp)
target_link_libraries(kinematic_bicycle matplotlib ${CASADI_LIBRARIES})

add_executable(wheeldyn_bicycle_mpc wheeldyn_bicycle_mpc.cpp)
target_link_libraries(wheeldyn_bicycle_mpc wheeldyn_bicycle Eigen3::Eigen jintegrator visualization_tools)

add_executable(simple_bicycle_mpc simple_bicycle_mpc.cpp)
target_link_libraries(simple_bicycle_mpc simple_bicycle Eigen3::Eigen jintegrator visualization_tools)

add_executable(kinematic_bicycle_mpc kinematic_bicycle_mpc.cpp)
target_link_libraries(kinematic_bicycle_mpc kinematic_bicycle Eigen3::Eigen jintegrator visualization_tools)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} jkite Eigen3::Eigen ${CASADI_LIBRARIES})

add_subdirectory(ThirdParty/polympc)

include_directories( ThirdParty/polympc/src ${CASADI_INCLUDE_DIR} ${Eigen3_INCLUDE_DIR} ${PYTHON_INCLUDE_DIRS})