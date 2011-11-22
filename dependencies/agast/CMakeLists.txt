cmake_minimum_required(VERSION 2.4.6)
    
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

add_definitions(-D_REENTRANT -Wall -O3 -march=nocona -Wextra -Winit-self -Woverloaded-virtual -Wnon-virtual-dtor -Wsign-promo -Wno-long-long )

#where the demo will be placed
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/installfiles)
#where the libs will be placed
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

file(GLOB AGAST_SOURCE_FILES  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")

file(GLOB AGAST_HEADER_FILES  "${CMAKE_CURRENT_SOURCE_DIR}/include/agast/*.h")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include/agast)

#build the library dynamic and static versions
rosbuild_add_library(agast ${AGAST_SOURCE_FILES} ${AGAST_HEADER_FILES})

#define some target ...
target_link_libraries(agast ${OpenCV_LIBS})
#add_library(agast STATIC ${AGAST_SOURCE_FILES} ${AGAST_HEADER_FILES})


