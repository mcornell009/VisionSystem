cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(calibrator)

#find_package(PCL 1.5 REQUIRED)

#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
#find_package(Boost REQUIRED)
SET(CMAKE_CXX_FLAGS -lboost_thread-mt)

#include_directories(${Boost_INCLUDE_DIRS})
#link_directories(${Boost_LIBRARY_DIRS})
#add_definitions(${Boost_DEFINITIONS})

add_executable (calibrator calibrator.cpp)
target_link_libraries (calibrator ${OpenCV_LIBS})

#target_link_libraries (calibrator ${Boost_LIBS})
#target_link_libraries (calibrator ${PCL_LIBRARIES})



#add_executable (RegionFinding RegionFinding.cpp)
#target_link_libraries (RegionFinding ${OpenCV_LIBS})
#target_link_libraries (RegionFinding ${Boost_LIBS})

#add_executable (ObjRec ObjRec.cpp)
#target_link_libraries (ObjRec ${OpenCV_LIBS})

