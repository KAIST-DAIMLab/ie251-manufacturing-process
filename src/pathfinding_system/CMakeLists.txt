cmake_minimum_required(VERSION 3.0.2)
project(pathfinding_system)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  message_generation
)

add_action_files(
  FILES
  MoveToNode.action
  FollowPath.action
)

add_message_files(
  FILES
  RobotState.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    actionlib
    actionlib_msgs
    message_runtime
)

install(PROGRAMS
  scripts/path_server_node
  scripts/robot_executor_node
  scripts/unpause_gazebo_node
  scripts/user_client
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_gazebo_control.py)
  catkin_add_nosetests(test/test_path_server.py)
  catkin_add_nosetests(test/test_turtlebot.py)
endif()
