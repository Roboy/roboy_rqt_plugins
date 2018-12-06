cmake_minimum_required(VERSION 2.8.3)
project( msj_platform_rqt )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g")

find_package(catkin REQUIRED COMPONENTS roscpp roslint rqt_gui rqt_gui_cpp std_msgs tf
        roboy_middleware_msgs common_utilities)
catkin_package(CATKIN_DEPENDS roboy_middleware_msgs common_utilities)

# Eigen
find_package(PkgConfig REQUIRED )
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories( include qcustomplot)

## System dependencies are found with CMake's conventions

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

roslint_cpp()

catkin_package( INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs roboy_middleware_msgs)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(msj_platform_rqt_MOCS include/msj_platform_rqt/msj_platform_rqt.hpp qcustomplot/qcustomplot.h)
  qt5_wrap_ui(msj_platform_rqt_UIS_H src/msj_platform_rqt.ui)
else()
  qt4_wrap_cpp(msj_platform_rqt_MOCS include/msj_platform_rqt/msj_platform_rqt.hpp qcustomplot/qcustomplot.h)
  qt4_wrap_ui(msj_platform_rqt_UIS_H src/msj_platform_rqt.ui)
endif()

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)

add_library( msj_platform_rqt src/msj_platform_rqt.cpp include/msj_platform_rqt/msj_platform_rqt.hpp
        ${msj_platform_rqt_MOCS} ${msj_platform_rqt_UIS_H} qcustomplot/qcustomplot.cpp)

add_dependencies(msj_platform_rqt roboy_middleware_msgs_gencpp)

target_link_libraries(msj_platform_rqt ${catkin_LIBRARIES} )

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(msj_platform_rqt Qt5::Widgets)
else()
  target_link_libraries(msj_platform_rqt ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

#############
## Install ##
#############

install(TARGETS msj_platform_rqt
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )