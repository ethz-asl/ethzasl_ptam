cmake_minimum_required(VERSION 2.4.6)
    
add_definitions(-D_REENTRANT -Wall -O3 -Wextra -Winit-self -Woverloaded-virtual -Wnon-virtual-dtor -Wsign-promo -Wno-long-long )

#where the demo will be placed
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/installfiles)
#where the libs will be placed
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

file(GLOB AGAST_SOURCE_FILES  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")

file(GLOB AGAST_HEADER_FILES  "${CMAKE_CURRENT_SOURCE_DIR}/include/agast/*.h")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include/agast)

#build the library dynamic and static versions
add_library(agast ${AGAST_SOURCE_FILES} ${AGAST_HEADER_FILES})


find_package(OpenCV REQUIRED)
target_link_libraries(agast ${OpenCV_LIBRARIES})


