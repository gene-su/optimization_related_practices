cmake_minimum_required(VERSION 3.2)

project(g2o_curve_fitting)

set(CMAKE_CXX_FLAGS "-std=c++17 -O3")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

add_executable(g2o_curve_fitting src/g2o_curve_fitting.cpp)
target_link_libraries(g2o_curve_fitting ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_DENSE})