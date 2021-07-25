cmake_minimum_required(VERSION 3.0.2)
project(roshand_control)

find_package(catkin REQUIRED COMPONENTS
  effort_controllers
  joint_state_controller
  robot_state_publisher
  roboticsgroup_upatras_gazebo_plugins
)

catkin_package(

)

include_directories(

  ${catkin_INCLUDE_DIRS}
)

