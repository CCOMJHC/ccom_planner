cmake_minimum_required(VERSION 2.8.3)
project(ccom_planner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  dubins_curves
  project11_navigation
  pluginlib
  roscpp
  tf2
)

catkin_package(
  LIBRARIES ccom_planner
  CATKIN_DEPENDS dubins_curves pluginlib project11_navigation roscpp tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


set(PLUGIN_SOURCES
  src/ccom_planner.cpp
  src/dubins_astar.cpp
)

add_library(ccom_planner ${PLUGIN_SOURCES})
add_dependencies(ccom_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ccom_planner
    ${catkin_LIBRARIES}
    project11_navigation
)

install(TARGETS ccom_planner
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
PATTERN ".svn" EXCLUDE
)

install(FILES plugin.xml
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
