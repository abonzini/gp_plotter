cmake_minimum_required(VERSION 3.0.2)
project(gp_plotter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
  controller_manager
)

add_service_files(
  FILES
  AddPoint.srv
  SetHyp.srv
  SetPrior.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES gp_plotter
  DEPENDS system_lib
  DEPENDS message_runtime
)

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
