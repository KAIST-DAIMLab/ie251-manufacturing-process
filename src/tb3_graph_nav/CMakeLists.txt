cmake_minimum_required(VERSION 3.0.2)
project(tb3_graph_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  tf
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs nav_msgs actionlib actionlib_msgs tf
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
