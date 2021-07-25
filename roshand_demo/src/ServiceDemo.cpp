#include <cstdlib>
#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include "roshand_msg/HandCommandWithoutSensor.h"
#include "roshand_msg/DisableSensor.h"
#include "roshand_msg/EnableSensor.h"

#include "roshand_msg/HandCommandWithSensor.h"
 
int main(int argc, char **argv)
{

  ros::init(argc, argv, "service_client_demo");
  
  

  ros::NodeHandle n;
  
  //std::string ns;
  //n.getParam("ns", ns);

//ns = "/roshand";

  ros::ServiceClient enable_sensor_touch_client = n.serviceClient<roshand_msg::EnableSensor>( "/enable_sensor_touch_service");
  ros::ServiceClient disable_sensor_touch_client = n.serviceClient<roshand_msg::DisableSensor>("/disable_sensor_touch_service");

  ros::ServiceClient command_without_sensor = n.serviceClient<roshand_msg::HandCommandWithoutSensor>( "/set_motor_target_position");

  ros::ServiceClient close_command_with_sensor = n.serviceClient<roshand_msg::HandCommandWithSensor>( "/close_hand_with_sensor_service");

  roshand_msg::HandCommandWithoutSensor command_without_sensor_;
  roshand_msg::DisableSensor sensor_disabled;
  roshand_msg::EnableSensor sensor_enabled;
  roshand_msg::HandCommandWithSensor command_with_sensor_;  


   ros::WallDuration(1).sleep();
   disable_sensor_touch_client.call(sensor_disabled);
   if(sensor_disabled.response.SENSOR_DISABLED == true)ROS_INFO("Sensor disabled when grasp................");

  while(command_without_sensor_.response.Command_Send == true)command_without_sensor_.response.Command_Send = false;
  ROS_INFO("Start Open Without Sensor");
  while(command_without_sensor_.response.Command_Send == false) {

       command_without_sensor_.request.target_turn = (uint32_t)130000;
       command_without_sensor.call(command_without_sensor_);
  }

  ros::WallDuration(2).sleep();

  ROS_INFO("Start Close Without Sensor");
  while(command_without_sensor_.response.Command_Send == true)command_without_sensor_.response.Command_Send = false;


  while(command_without_sensor_.response.Command_Send == false) {

       command_without_sensor_.request.target_turn = (uint32_t)2000;
       command_without_sensor.call(command_without_sensor_);
  }


  ros::WallDuration(2).sleep();

   while(command_without_sensor_.response.Command_Send == true)command_without_sensor_.response.Command_Send = false;
   ROS_INFO("Start Open Without Sensor");

   while(command_without_sensor_.response.Command_Send == false) {

       command_without_sensor_.request.target_turn = (uint32_t)130000;
       command_without_sensor.call(command_without_sensor_);
  }




  ros::WallDuration(2).sleep();

  enable_sensor_touch_client.call(sensor_enabled);
  if(sensor_enabled.response.SENSOR_ENABLED == true)ROS_INFO("Sensor enabled when grasp.........");


  ROS_INFO("Start Close with sensor.............");
  while(command_with_sensor_.response.Command_Send == true)command_with_sensor_.response.Command_Send = false;
  while(command_with_sensor_.response.Command_Send == false){
  command_with_sensor_.request.close_step_mag = (uint8_t)12;

  command_with_sensor_.request.open_step_mag = (uint8_t)1;
  close_command_with_sensor.call(command_with_sensor_);
  }


  ros::WallDuration(2).sleep();

  disable_sensor_touch_client.call(sensor_disabled);
  if(sensor_disabled.response.SENSOR_DISABLED == true)ROS_INFO("Sensor disabled when grasp.............");

  ROS_INFO("Start Open Without Sensor");
  while(command_without_sensor_.response.Command_Send == true)command_without_sensor_.response.Command_Send = false;
  while(command_without_sensor_.response.Command_Send == false) {

       command_without_sensor_.request.target_turn = (uint32_t)130000;
       command_without_sensor.call(command_without_sensor_);
  }
  ros::WallDuration(2).sleep();
 
  enable_sensor_touch_client.call(sensor_enabled);
  if(sensor_enabled.response.SENSOR_ENABLED == true)ROS_INFO("Sensor enabled when grasp.........");



  return 0;
}
