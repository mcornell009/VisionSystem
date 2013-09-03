cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(ObjectMatching)

find_package(OpenCV REQUIRED)

add_executable (ObjRec ObjRec.cpp)

target_link_libraries (ObjRec ${OpenCV_LIBS})

add_executable (ObjRecClass ObjRecClass.cpp)

target_link_libraries (ObjRecClass ${OpenCV_LIBS})
