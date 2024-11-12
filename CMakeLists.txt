cmake_minimum_required(VERSION 3.0)
project(PointCloudDenoising)

SET(CMAKE_BUILD_TYPE "Debug")

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(statistical_filtering statistical_filtering.cpp)
add_executable(radius_filtering radius_filtering.cpp)
add_executable(passthrough_filtering passthrough_filtering.cpp)
add_executable(conditional_filtering conditional_filtering.cpp)
add_executable(projectmodel_filtering projectmodel_filtering.cpp)
add_executable(voxel_filtering voxel_filtering.cpp)
add_executable(bilateral_filtering bilateral_filtering.cpp)
add_executable(gaussian_filtering gaussian_filtering.cpp)

target_link_libraries(statistical_filtering ${PCL_LIBRARIES})
target_link_libraries(radius_filtering ${PCL_LIBRARIES})
target_link_libraries(passthrough_filtering ${PCL_LIBRARIES})
target_link_libraries(conditional_filtering ${PCL_LIBRARIES})
target_link_libraries(projectmodel_filtering ${PCL_LIBRARIES})
target_link_libraries(voxel_filtering ${PCL_LIBRARIES})
target_link_libraries(bilateral_filtering ${PCL_LIBRARIES})
target_link_libraries(gaussian_filtering ${PCL_LIBRARIES})