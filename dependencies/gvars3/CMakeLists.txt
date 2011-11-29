cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#get TooN path
execute_process(COMMAND rospack find TooN RESULT_VARIABLE _TooN_failed OUTPUT_VARIABLE TooNPATH)
#get libcvd path
execute_process(COMMAND rospack find libcvd RESULT_VARIABLE _libcvd_failed OUTPUT_VARIABLE libcvdPATH)

if("${_TooN_failed}" STREQUAL 0)
	if("${_libcvd_failed}" STREQUAL 0)
		#export variables
		string(STRIP ${TooNPATH} TooNPATH)
		string(STRIP ${libcvdPATH} libcvdPATH)
        set(ENV{CPPFLAGS} -I${TooNPATH}/include -I${libcvdPATH}/include)

    # copy install files for easier (and cleaner ;) ) make clean
		execute_process(WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} COMMAND cp -rup ${PROJECT_SOURCE_DIR}/installfiles/ ${PROJECT_SOURCE_DIR}/build RESULT_VARIABLE _make_ok)

    #run configure
		execute_process(WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/build/installfiles COMMAND ./configure --disable-widgets --prefix=${PROJECT_SOURCE_DIR} RESULT_VARIABLE _make_ok)
		if(NOT (${_make_ok} STREQUAL 0))
            message(FATAL_ERROR "configuring gvars failed")
        endif(NOT (${_make_ok} STREQUAL 0))
    
		execute_process(WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/build/installfiles COMMAND make  RESULT_VARIABLE _make_ok)
		if(NOT (${_make_ok} STREQUAL 0))
            message(FATAL_ERROR "building gvars failed")
        endif(NOT (${_make_ok} STREQUAL 0))
        
		execute_process(WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/build/installfiles COMMAND make install  RESULT_VARIABLE _make_ok)
		
	else("${_libcvd_failed}" STREQUAL 0)
		message(FATA_ERROR ROS libcvd package not found! ${libcvdPATH})
	endif("${_libcvd_failed}" STREQUAL 0)
else("${_TooN_failed}" STREQUAL 0)
	message(FATA_ERROR ROS TooN package not found! ${TooNPATH})
endif("${_TooN_failed}" STREQUAL 0)

# add directories that should be removed on make clean
list(APPEND GVARS_MAKE_CLEAN_FILES 
  ${PROJECT_SOURCE_DIR}/include 
  ${PROJECT_SOURCE_DIR}/bin 
  ${PROJECT_SOURCE_DIR}/lib 
)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${GVARS_MAKE_CLEAN_FILES}")
