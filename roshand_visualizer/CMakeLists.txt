cmake_minimum_required(VERSION 3.0.2)
project(roshand_visualizer)


 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roshand_msg
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
)

include_directories(
${catkin_INCLUDE_DIRS}
)


add_executable(roshand_visual src/roshand_visual.cpp )
add_dependencies(roshand_visual ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} roshand_msg_gencpp) 
target_link_libraries(roshand_visual  ${catkin_LIBRARIES})



