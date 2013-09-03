cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(facedetect)

#find_package(PCL 1.5 REQUIRED)

#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)

add_executable (facedetect_camera facedetect_camera.cpp)
#target_link_libraries (facedetect_camera ${PCL_LIBRARIES})
target_link_libraries (facedetect_camera ${OpenCV_LIBS})

add_executable (facedetect_servo facedetect_servo.cpp)
