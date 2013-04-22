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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

set(rqt_ptam_SRCS
  src/rqt_ptam/remote_ptam.cpp
  src/rqt_ptam/ratio_layouted_frame.cpp
)

set(rqt_ptam_HDRS
  include/rqt_ptam/remote_ptam.h
  include/rqt_ptam/ratio_layouted_frame.h
)

set(rqt_ptam_UIS
  src/rqt_ptam/remote_ptam.ui
)

set(rqt_ptam_INCLUDE_DIRECTORIES
#  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

qt4_wrap_cpp(rqt_ptam_MOCS ${rqt_ptam_HDRS} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
qt4_wrap_ui(rqt_ptam_UIS_H ${rqt_ptam_UIS})

include_directories(${rqt_ptam_INCLUDE_DIRECTORIES} )
rosbuild_add_library(rqt_ptam ${rqt_ptam_SRCS} ${rqt_ptam_MOCS} ${rqt_ptam_UIS_H})
target_link_libraries(rqt_ptam ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})


