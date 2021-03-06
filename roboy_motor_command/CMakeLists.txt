cmake_minimum_required(VERSION 2.8.3)
project( roboy_motor_command)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g")

find_package(catkin REQUIRED COMPONENTS common_utilities roscpp roslint rqt_gui rqt_gui_cpp std_msgs roboy_middleware_msgs )
FIND_PACKAGE(yaml-cpp REQUIRED)

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

catkin_package( INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs roboy_middleware_msgs common_utilities 
        DEPENDS YAML_CPP)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(roboy_motor_command_MOCS include/roboy_motor_command/roboy_motor_command.hpp qcustomplot/qcustomplot.h)
  qt5_wrap_ui(roboy_motor_command_UIS_H src/roboy_motor_command.ui)
else()
  qt4_wrap_cpp(roboy_motor_command_MOCS include/roboy_motor_command/roboy_motor_command.hpp qcustomplot/qcustomplot.h)
  qt4_wrap_ui(roboy_motor_command_UIS_H src/roboy_motor_command.ui)
endif()

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_library( roboy_motor_command src/roboy_motor_command.cpp ${roboy_motor_command_MOCS} ${roboy_motor_command_UIS_H}
        qcustomplot/qcustomplot.cpp)

add_dependencies(roboy_motor_command roboy_middleware_msgs_gencpp)

target_link_libraries(roboy_motor_command ${catkin_LIBRARIES}  ${YAML_CPP_LIBRARIES} -ldl)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(roboy_motor_command Qt5::Widgets)
else()
  target_link_libraries(roboy_motor_command ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

#############
## Install ##
#############

install(TARGETS roboy_motor_command
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/roboy_rqt_plugins
        )

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )