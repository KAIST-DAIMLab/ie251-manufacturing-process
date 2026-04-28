cmake_minimum_required(VERSION 3.0.2)
project(tb3_graph_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  tf
  message_generation
)

add_message_files(FILES NodeStatus.msg)
add_action_files(FILES NavigateToNode.action)

catkin_python_setup()

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs nav_msgs actionlib actionlib_msgs tf message_runtime
)

catkin_install_python(PROGRAMS
  scripts/position_reporter.py
  scripts/nav_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
