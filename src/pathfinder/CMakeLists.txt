cmake_minimum_required(VERSION 3.0.2)
project(pathfinder)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  message_generation
)

add_action_files(
  FILES
  MoveToNode.action
  FollowPath.action
  RobotCommand.action
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
    sensor_msgs
    actionlib
    actionlib_msgs
    message_runtime
)

install(PROGRAMS
  scripts/path_server_node
  scripts/robot_executors_node
  scripts/spawn_simulation_robots
  scripts/spawn_simulation_markers
  scripts/client
  scripts/run_simulation
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_launch_split.py)
  catkin_add_nosetests(test/test_collision_monitor.py)
  catkin_add_nosetests(test/test_motion.py)
  catkin_add_nosetests(test/test_path_server.py)
  catkin_add_nosetests(test/test_physics_utils.py)
  catkin_add_nosetests(test/test_turtlebot.py)
  catkin_add_nosetests(test/test_client.py)
  catkin_add_nosetests(test/test_obstacle_detector.py)
endif()
