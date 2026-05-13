cmake_minimum_required(VERSION 3.0.2)
project(pathfinder)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  actionlib
  actionlib_msgs
  message_generation
)

add_action_files(
  FILES
  FollowPath.action
  RobotCommand.action
)

add_service_files(
  FILES
  MoveToNode.srv
  CancelPath.srv
  RotateRobot.srv
  GetGraph.srv
  GetRobots.srv
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
    tf2_ros
    actionlib
    actionlib_msgs
    message_runtime
)

install(PROGRAMS
  scripts/path_server
  scripts/robot
  scripts/simulation_robots
  scripts/simulation_markers
  scripts/client
  scripts/run_simulation
  scripts/real_robot_launch
  scripts/real_robot_localization
  scripts/real_odom_tf_bridge
  scripts/simulation
  scripts/scan_relay
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_launch_split.py)
  catkin_add_nosetests(test/test_motion.py)
  catkin_add_nosetests(test/test_fleet_service.py)
  catkin_add_nosetests(test/test_motion_control_action_server.py)
  catkin_add_nosetests(test/test_path_orchestrator.py)
  catkin_add_nosetests(test/test_path_follow_action_client.py)
  catkin_add_nosetests(test/test_physics_utils.py)
  catkin_add_nosetests(test/test_turtlebot.py)
  catkin_add_nosetests(test/test_client.py)
  catkin_add_nosetests(test/test_obstacle_detector.py)
  catkin_add_nosetests(test/test_real_robot_launch.py)
  catkin_add_nosetests(test/test_real_robot_localization.py)
  catkin_add_nosetests(test/test_real_odom_tf_bridge.py)
endif()
