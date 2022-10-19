cmake_minimum_required(VERSION 2.8.3)
project(orbit)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  geographic_msgs
  project11
  project11_navigation
  tf
  pluginlib
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  geographic_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES orbit
  CATKIN_DEPENDS pluginlib project11 project11_navigation roscpp tf2 tf2_ros 
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


set(PLUGIN_SOURCES
  src/orbit_plugin.cpp
  src/orbit_task.cpp
)

add_library(orbit_plugins ${PLUGIN_SOURCES})
add_dependencies(orbit_plugins ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(orbit_plugins
    ${catkin_LIBRARIES}
    project11_navigation
    )

install(TARGETS orbit_plugins 
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
PATTERN ".svn" EXCLUDE
)

install(FILES project11_navigation_plugin.xml
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
