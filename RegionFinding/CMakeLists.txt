cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(Disparity_Comparison)

find_package(OpenCV REQUIRED)

add_executable (RegionFindingSingle RegionFindingSingle.cpp)

target_link_libraries (RegionFindingSingle ${OpenCV_LIBS})

add_executable (RegionFindingLive RegionFindingLive.cpp)

target_link_libraries (RegionFindingLive ${OpenCV_LIBS})
