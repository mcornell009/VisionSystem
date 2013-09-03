cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(DBcreator)

find_package(OpenCV REQUIRED)

SET(CMAKE_CXX_FLAGS "-lboost_thread-mt")

add_executable (DBcreator DBcreator.cpp)
target_link_libraries (DBcreator ${OpenCV_LIBS})

