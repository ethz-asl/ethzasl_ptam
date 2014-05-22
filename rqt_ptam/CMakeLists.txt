cmake_minimum_required(VERSION 2.8.3)
project(rqt_ptam)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
find_package(catkin REQUIRED)


catkin_package()

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

include_directories(include ${rqt_ptam_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(rqt_ptam ${rqt_ptam_SRCS} ${rqt_ptam_MOCS} ${rqt_ptam_UIS_H})
target_link_libraries(rqt_ptam ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${catkin_LIBRARIES})


