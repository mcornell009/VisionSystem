cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(Tracking)

find_package(OpenCV REQUIRED)
#find_package(Boost REQUIRED)

#include_directories(${Boost_INCLUDE_DIRS})
#link_directories(${Boost_LIBRARY_DIRS})
#add_definitions(${Boost_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "-lboost_thread-mt -lphidget21")

add_executable (Tracking Tracking.cpp)
target_link_libraries (Tracking ${OpenCV_LIBS})

#add_executable (motor_control motor_control.cpp)
#target_link_libraries (motor_control ${OpenCV_LIBS})




#add_executable (ObjRec ObjRec.cpp)
#target_link_libraries (ObjRec ${OpenCV_LIBS})

