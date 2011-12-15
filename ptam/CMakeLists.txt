cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Generate dynamic parameters
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

set(PTAM_SOURCE_FILES 
#	src/ARDriver.cc           
	src/GLWindow2.cc       
	src/main.cc      
	src/MapViewer.cc 
	src/ATANCamera.cc          
	src/GLWindowMenu.cc    
	src/Map.cc       
	src/MiniPatch.cc    
	src/ShiTomasi.cc         
	src/Tracker.cc
    src/Bundle.cc                 
    src/HomographyInit.cc  
    src/MapMaker.cc  
    src/PatchFinder.cc  
    src/SmallBlurryImage.cc                
    src/KeyFrame.cc        
    src/MapPoint.cc  
    src/Relocaliser.cc  
    src/System.cc     
    src/Params.cpp
	)

set(PTAM_HEADER_FILES
 #   include/ptam/ARDriver.h          
    include/ptam/CameraCalibrator.h  
    include/ptam/KeyFrame.h      
    include/ptam/MapViewer.h    
    include/ptam/Relocaliser.h       
    include/ptam/TrackerData.h
    include/ptam/ATANCamera.h         
    include/ptam/LevelHelpers.h  
    include/ptam/MEstimator.h   
    include/ptam/ShiTomasi.h         
    include/ptam/Tracker.h
    include/ptam/Bundle.h            
    include/ptam/GLWindow2.h         
    include/ptam/Map.h           
    include/ptam/MiniPatch.h    
    include/ptam/SmallBlurryImage.h 
    include/ptam/CalibCornerPatch.h  
    include/ptam/GLWindowMenu.h      
    include/ptam/MapMaker.h      
    include/ptam/OpenGL.h       
    include/ptam/SmallMatrixOpts.h
    include/ptam/CalibImage.h        
    include/ptam/HomographyInit.h    
    include/ptam/MapPoint.h      
    include/ptam/PatchFinder.h  
    include/ptam/System.h  
    include/ptam/Params.h
	)
	
SET(CALIBRATOR_FILES     
    src/CameraCalibrator.cc 
	src/GLWindowMenu.cc 
	src/CalibCornerPatch.cc           
	src/CalibImage.cc            
	src/GLWindow2.cc	
	src/ATANCamera.cc 
    src/Params.cpp 
	)	
	


#include_directories(/usr/local/include) 
set(PTAM_LIBRARIES GL glut)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()

add_definitions(-DKF_REPROJ)

# PTAM executable
rosbuild_add_executable(ptam ${PTAM_SOURCE_FILES} ${PTAM_HEADER_FILES})
rosbuild_add_compile_flags(ptam -D_LINUX -D_REENTRANT -Wall  -O3 -march=nocona -msse3)
target_link_libraries(ptam ${PTAM_LIBRARIES})
#rosbuild_link_boost(ptam thread signals system)

# Camera calibrator 
rosbuild_add_executable(cameracalibrator ${CALIBRATOR_FILES})
target_link_libraries(cameracalibrator ${PTAM_LIBRARIES})

# PTAM remote control
rosbuild_add_executable(remote_ptam src/RemotePtam.cpp)


rosbuild_add_executable(pointcloudpublisher src/PTAMVisualizer.cpp)
