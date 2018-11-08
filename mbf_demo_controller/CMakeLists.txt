cmake_minimum_required(VERSION 2.8.3)
project(mbf_demo_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mbf_costmap_core
  mbf_msgs
  pluginlib
  tf
  costmap_2d
)

include_directories(
  include
 ${catkin_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp mbf_costmap_core mbf_msgs pluginlib tf costmap_2d
)

add_library(${PROJECT_NAME} src/mbf_demo_controller.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES mbf_demo_controller.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
