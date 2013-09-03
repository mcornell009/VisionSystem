cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(Disparity_Comparison)

#find_package(PCL 1.5 REQUIRED)

#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)

add_executable (disparity disparity.cpp)
target_link_libraries (disparity ${OpenCV_LIBS})

#add_executable (disp_compare disp_compare.cpp)

#target_link_libraries (disp_compare ${PCL_LIBRARIES})
#target_link_libraries (disp_compare ${OpenCV_LIBS})
