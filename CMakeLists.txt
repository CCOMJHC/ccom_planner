cmake_minimum_required(VERSION 2.8.3)
project(ccom_planner)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  behaviortree_cpp
  dubins_curves
  project11_navigation
  pluginlib
  roscpp
  tf2
)

catkin_package(
  LIBRARIES ccom_planner_bt_plugin
  CATKIN_DEPENDS behaviortree_cpp dubins_curves pluginlib project11_navigation roscpp tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


set(PLUGIN_SOURCES
  src/dubins_astar.cpp
  src/plan_action.cpp
)

add_library(ccom_planner_bt_plugin SHARED ${PLUGIN_SOURCES})
target_link_libraries(ccom_planner_bt_plugin PRIVATE ${BTCPP_LIBRARY}
    ${catkin_LIBRARIES}
    #project11_navigation
)

target_compile_definitions(ccom_planner_bt_plugin PRIVATE  BT_PLUGIN_EXPORT )

install(TARGETS ccom_planner_bt_plugin
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
