cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(Pipeline)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_executable(Pipeline Pipeline.cpp)
add_library(Stages base.hpp utility.hpp utility.cpp Stage1.hpp Stage1.cpp Stage2.hpp Stage2.cpp Stage3.hpp Stage3.cpp Stage4.hpp Stage4.cpp Stage5.hpp Stage5.cpp)

target_link_libraries(Pipeline ${PCL_LIBRARIES} Stages)
