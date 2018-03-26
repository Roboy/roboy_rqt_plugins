cmake_minimum_required(VERSION 2.8.3)
project( roboy_trajectories_control)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g")

find_package(catkin REQUIRED COMPONENTS common_utilities roscpp roslint rqt_gui rqt_gui_cpp std_msgs
        roboy_communication_middleware roboy_communication_control)
# FIND_PACKAGE(yaml-cpp REQUIRED)

 include_directories( include )

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
        CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs roboy_communication_middleware  roboy_communication_control
        common_utilities
        DEPENDS YAML_CPP)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(roboy_trajectories_control_MOCS include/roboy_trajectories_control/roboy_trajectories_control.hpp )
  qt5_wrap_ui(roboy_trajectories_control_UIS_H resource/roboy_trajectories_control.ui)
else()
  qt4_wrap_cpp(roboy_trajectories_control_MOCS include/roboy_trajectories_control/roboy_trajectories_control.hpp )
  qt4_wrap_ui(roboy_trajectories_control_UIS_H resource/roboy_trajectories_control.ui)
endif()

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_library( roboy_trajectories_control src/roboy_trajectories_control.cpp ${roboy_trajectories_control_MOCS} ${roboy_trajectories_control_UIS_H}
)

add_dependencies(roboy_trajectories_control roboy_communication_middleware_gencpp roboy_communication_control_gencpp)

target_link_libraries(roboy_trajectories_control ${catkin_LIBRARIES} )

# add_executable( speed_test src/speed_test.cpp)
# add_dependencies(speed_test roboy_communication_middleware_gencpp)
# target_link_libraries(speed_test ${catkin_LIBRARIES} )
# target_link_libraries(roboy_trajectories_control ${YAML_CPP_LIBRARIES} -ldl)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(roboy_trajectories_control Qt5::Widgets)
else()
  target_link_libraries(roboy_trajectories_control ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

#############
## Install ##
#############

install(TARGETS roboy_trajectories_control
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/roboy_rqt_plugins
        )

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )