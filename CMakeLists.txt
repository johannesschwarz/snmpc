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

add_library(mobile_robot_new mobile_robot_new.cpp mobile_robot_new.h)
target_link_libraries(mobile_robot_new matplotlib ${CASADI_LIBRARIES})

add_library(kinematic_robot kinematic_robot.cpp kinematic_robot.hpp)
target_link_libraries(kinematic_robot matplotlib mobile_robot_new ${CASADI_LIBRARIES})

add_executable(mobile_robot_test mobile_robot_test.cpp)
target_link_libraries(mobile_robot_test mobile_robot_new Eigen3::Eigen jintegrator visualization_tools)

add_executable(kinematic_robot_test kinematic_robot_test.cpp)
target_link_libraries(kinematic_robot_test kinematic_robot Eigen3::Eigen jintegrator visualization_tools)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} jkite Eigen3::Eigen ${CASADI_LIBRARIES})

add_subdirectory(ThirdParty/polympc)

include_directories( ThirdParty/polympc/src ${CASADI_INCLUDE_DIR} ${Eigen3_INCLUDE_DIR} ${PYTHON_INCLUDE_DIRS})