cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(surface_rec)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (greedy_projection greedy_projection.cpp)
add_executable (resample resample.cpp)
add_executable (surfaceRec surfaceRec.cpp)
target_link_libraries (greedy_projection ${PCL_LIBRARIES})
target_link_libraries (resample ${PCL_LIBRARIES})
target_link_libraries (surfaceRec ${PCL_LIBRARIES})
