cmake_minimum_required(VERSION 3.0)
project(PointCloudDenoising)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(statistical_filtering statistical_filtering.cpp)

target_link_libraries(statistical_filtering ${PCL_LIBRARIES})