cmake_minimum_required(VERSION 3.0.2)
project(approx_manifold_prm)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  moveit_msgs
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
)

add_service_files(
  FILES
  PlanTrajectory.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  moveit_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    trajectory_msgs
    moveit_ros_planning_interface
    moveit_msgs
    tf2_ros
    tf2_geometry_msgs
    message_runtime
)

catkin_install_python(
  PROGRAMS
    scripts/approx_manifold_graph.py
    scripts/prm_star.py
    scripts/lazy_prm.py
    scripts/trajectory_planner_node.py
    scripts/build_approx_graph.py
    scripts/build_roadmap.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
