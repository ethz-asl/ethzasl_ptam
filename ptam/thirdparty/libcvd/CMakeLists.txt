cmake_minimum_required(VERSION 2.4.6)

set(TooNPATH ${PROJECT_SOURCE_DIR}/thirdparty/TooN)
set(_TooN_failed 0)

if("${_TooN_failed}" STREQUAL 0)
	#export variables
    string(STRIP ${TooNPATH} TooNPATH)
    set(ENV{CPPFLAGS} -I${TooNPATH}/include)
    set(ENV{CXXFLAGS} -D_REENTRANT)

   # copy install files for easier (and cleaner ;) ) make clean
	execute_process(WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} COMMAND mkdir ${CMAKE_CURRENT_SOURCE_DIR}/build RESULT_VARIABLE _make_ok)
	execute_process(WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} COMMAND cp -rup ${CMAKE_CURRENT_SOURCE_DIR}/installfiles/ ${CMAKE_CURRENT_SOURCE_DIR}/build RESULT_VARIABLE _make_ok)

	#run configure
	execute_process(WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/installfiles COMMAND ./configure --without-ffmpeg --without-dc1394v1 --without-dc1394v2 --with-lapack --prefix=${CMAKE_CURRENT_SOURCE_DIR}  RESULT_VARIABLE _make_ok)
	if(NOT (${_make_ok} STREQUAL 0))
        message(FATAL_ERROR "configuring libcvd failed")
    endif(NOT (${_make_ok} STREQUAL 0))
	
	# make
	execute_process(WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/installfiles COMMAND make  RESULT_VARIABLE _make_ok)
	if(NOT (${_make_ok} STREQUAL 0))
        message(FATAL_ERROR "compiling libcvd failed")
    endif(NOT (${_make_ok} STREQUAL 0))
	
	#make install
	execute_process(WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/installfiles COMMAND make install  RESULT_VARIABLE _make_ok)
	
else("${_TooN_failed}" STREQUAL 0)
	message(FATAL_ERROR ROS TooN package not found! ${TooNPATH})
endif("${_TooN_failed}" STREQUAL 0)

# add directories that should be removed on make clean
list(APPEND LIBCVD_MAKE_CLEAN_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/bin ${CMAKE_CURRENT_SOURCE_DIR}/lib ${CMAKE_CURRENT_SOURCE_DIR}/build)
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${LIBCVD_MAKE_CLEAN_FILES}")
