cmake_minimum_required(VERSION 3.2)

project(keypoint_matcher)

set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS "-Wall")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(OpenCV 4.7 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable(ceres_curve_fitting src/ceres_curve_fitting.cpp)
target_link_libraries(ceres_curve_fitting ${CERES_LIBRARIES} ${OpenCV_LIBRARIES})