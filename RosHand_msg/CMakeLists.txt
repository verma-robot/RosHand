cmake_minimum_required(VERSION 3.0.2)
project(roshand_msg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  message_generation
  actionlib
)

 add_message_files(
   FILES
   FingerPressure.msg
   Motor.msg
   Hand.msg

 )


 add_service_files(
   FILES
   HandCommandWithoutSensor.srv
   SensorCalibrate.srv
   CommandUseSensor.srv
   HandCommandWithSensor.srv
   DisableSensor.srv
   EnableSensor.srv

 )


 add_action_files(
   FILES
   CommandMotorWithSensor.action
   CommandMotorWithoutSensor.action

 )


 generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
 )

catkin_package(


  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs

)

include_directories(

  ${catkin_INCLUDE_DIRS}
)

