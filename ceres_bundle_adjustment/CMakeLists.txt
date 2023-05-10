cmake_minimum_required(VERSION 3.2)

project(ceres_bundle_adjustment)

set(CMAKE_CXX_FLAGS "-O3 -std=c++17")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable(ceres_bundle_adjustment src/ceres_bundle_adjustment.cpp)
target_link_libraries(ceres_bundle_adjustment ${CERES_LIBRARIES})
