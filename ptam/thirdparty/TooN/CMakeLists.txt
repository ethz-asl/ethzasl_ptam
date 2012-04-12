cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# generate documentation
execute_process(WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc COMMAND doxygen Doxyfile RESULT_VARIABLE _make_failed)

# add directories that should be removed on make clean
list(APPEND TOON_MAKE_CLEAN_FILES ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
message("${TOON_MAKE_CLEAN_FILES}")
set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${TOON_MAKE_CLEAN_FILES}")
