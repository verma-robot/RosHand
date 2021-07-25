
#include <string>

#include <ros/ros.h>
#include <ros/time.h>

#include "roshand_msg/Hand.h"
#include "roshand_msg/FingerPressure.h"
#include "roshand_msg/Motor.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "sensor_msgs/JointState.h"

#include <robot_state_publisher/robot_state_publisher.h>

using namespace std;

ros::Publisher joint_pub;
ros::Publisher sensor_pub;
//

void publish_sensors_to_rviz(const roshand_msg::HandConstPtr& hand) {
  bool contact_val;
  float pressure_val;
  double up_position;
  double sqrt_up;
  double up_si;


  visualization_msgs::MarkerArray marker_array;
  sensor_msgs::JointState joint_state;
  for (int finger=0; finger<4; finger++)
  {
    
    for (int i = 0; i < 8; i++)    
    {

      float radius = 0.004;
      float height = 0.005;
      visualization_msgs::Marker pressure_marker;
      visualization_msgs::Marker contact_marker;

      contact_val = hand->finger[finger].contact[i];
      pressure_val = hand->finger[finger].sensor[i];


      if(finger==0 && i==0)contact_marker.header.frame_id="left_fingertips_sensor_1_link";
      if(finger==0 && i==1)contact_marker.header.frame_id="left_fingertips_sensor_2_link";
      if(finger==0 && i==2)contact_marker.header.frame_id="left_fingertips_sensor_3_link";
      if(finger==0 && i==3)contact_marker.header.frame_id="left_fingertips_sensor_4_link";
      if(finger==0 && i==4)contact_marker.header.frame_id="left_fingertips_sensor_5_link";
      if(finger==0 && i==5)contact_marker.header.frame_id="left_fingertips_sensor_6_link";
      if(finger==0 && i==6)contact_marker.header.frame_id="left_fingertips_sensor_7_link";
      if(finger==0 && i==7)contact_marker.header.frame_id="left_fingertips_sensor_8_link";

      if(finger==1 && i==0)contact_marker.header.frame_id="left_sensor_1_link";
      if(finger==1 && i==1)contact_marker.header.frame_id="left_sensor_2_link";
      if(finger==1 && i==2)contact_marker.header.frame_id="left_sensor_3_link";
      if(finger==1 && i==3)contact_marker.header.frame_id="left_sensor_4_link";
      if(finger==1 && i==4)contact_marker.header.frame_id="left_sensor_5_link";
      if(finger==1 && i==5)contact_marker.header.frame_id="left_sensor_6_link";
      if(finger==1 && i==6)contact_marker.header.frame_id="left_sensor_7_link";
      if(finger==1 && i==7)contact_marker.header.frame_id="left_sensor_8_link";

      if(finger==2 && i==0)contact_marker.header.frame_id="right_fingertips_sensor_1_link";
      if(finger==2 && i==1)contact_marker.header.frame_id="right_fingertips_sensor_2_link";
      if(finger==2 && i==2)contact_marker.header.frame_id="right_fingertips_sensor_3_link";
      if(finger==2 && i==3)contact_marker.header.frame_id="right_fingertips_sensor_4_link";
      if(finger==2 && i==4)contact_marker.header.frame_id="right_fingertips_sensor_5_link";
      if(finger==2 && i==5)contact_marker.header.frame_id="right_fingertips_sensor_6_link";
      if(finger==2 && i==6)contact_marker.header.frame_id="right_fingertips_sensor_7_link";
      if(finger==2 && i==7)contact_marker.header.frame_id="right_fingertips_sensor_8_link";

      if(finger==3 && i==0)contact_marker.header.frame_id="right_sensor_1_link";
      if(finger==3 && i==1)contact_marker.header.frame_id="right_sensor_2_link";
      if(finger==3 && i==2)contact_marker.header.frame_id="right_sensor_3_link";
      if(finger==3 && i==3)contact_marker.header.frame_id="right_sensor_4_link";
      if(finger==3 && i==4)contact_marker.header.frame_id="right_sensor_5_link";
      if(finger==3 && i==5)contact_marker.header.frame_id="right_sensor_6_link";
      if(finger==3 && i==6)contact_marker.header.frame_id="right_sensor_7_link";
      if(finger==3 && i==7)contact_marker.header.frame_id="right_sensor_8_link";

      contact_marker.ns = "contact_markers";
    

     if (contact_val) {
          contact_marker.scale.x = radius;
          contact_marker.scale.y = radius;
          contact_marker.scale.z = height;
          contact_marker.color.r = 1.0;
          contact_marker.color.g = 0.0;
          contact_marker.color.b = 0.0;
          contact_marker.color.a = 1.0;

     }
     else {
          contact_marker.scale.x = radius / 2;
          contact_marker.scale.y = radius / 2;
          contact_marker.scale.z = height - 0.001;
          contact_marker.color.r = 1.0;
          contact_marker.color.g = 1.0;
          contact_marker.color.b = 1.0;
          contact_marker.color.a = 1.0;
     }
     contact_marker.pose.position.x=0.012;
     contact_marker.pose.position.y=0;
     contact_marker.pose.position.z=0.0027;


      contact_marker.pose.orientation.y=0.707;
      contact_marker.pose.orientation.w=0.707;



      if(finger==0 && i==0)pressure_marker.header.frame_id="left_fingertips_sensor_1_link";
      if(finger==0 && i==1)pressure_marker.header.frame_id="left_fingertips_sensor_2_link";
      if(finger==0 && i==2)pressure_marker.header.frame_id="left_fingertips_sensor_3_link";
      if(finger==0 && i==3)pressure_marker.header.frame_id="left_fingertips_sensor_4_link";
      if(finger==0 && i==4)pressure_marker.header.frame_id="left_fingertips_sensor_5_link";
      if(finger==0 && i==5)pressure_marker.header.frame_id="left_fingertips_sensor_6_link";
      if(finger==0 && i==6)pressure_marker.header.frame_id="left_fingertips_sensor_7_link";
      if(finger==0 && i==7)pressure_marker.header.frame_id="left_fingertips_sensor_8_link";

      if(finger==1 && i==0)pressure_marker.header.frame_id="left_sensor_1_link";
      if(finger==1 && i==1)pressure_marker.header.frame_id="left_sensor_2_link";
      if(finger==1 && i==2)pressure_marker.header.frame_id="left_sensor_3_link";
      if(finger==1 && i==3)pressure_marker.header.frame_id="left_sensor_4_link";
      if(finger==1 && i==4)pressure_marker.header.frame_id="left_sensor_5_link";
      if(finger==1 && i==5)pressure_marker.header.frame_id="left_sensor_6_link";
      if(finger==1 && i==6)pressure_marker.header.frame_id="left_sensor_7_link";
      if(finger==1 && i==7)pressure_marker.header.frame_id="left_sensor_8_link";

      if(finger==2 && i==0)pressure_marker.header.frame_id="right_fingertips_sensor_1_link";
      if(finger==2 && i==1)pressure_marker.header.frame_id="right_fingertips_sensor_2_link";
      if(finger==2 && i==2)pressure_marker.header.frame_id="right_fingertips_sensor_3_link";
      if(finger==2 && i==3)pressure_marker.header.frame_id="right_fingertips_sensor_4_link";
      if(finger==2 && i==4)pressure_marker.header.frame_id="right_fingertips_sensor_5_link";
      if(finger==2 && i==5)pressure_marker.header.frame_id="right_fingertips_sensor_6_link";
      if(finger==2 && i==6)pressure_marker.header.frame_id="right_fingertips_sensor_7_link";
      if(finger==2 && i==7)pressure_marker.header.frame_id="right_fingertips_sensor_8_link";

      if(finger==3 && i==0)pressure_marker.header.frame_id="right_sensor_1_link";
      if(finger==3 && i==1)pressure_marker.header.frame_id="right_sensor_2_link";
      if(finger==3 && i==2)pressure_marker.header.frame_id="right_sensor_3_link";
      if(finger==3 && i==3)pressure_marker.header.frame_id="right_sensor_4_link";
      if(finger==3 && i==4)pressure_marker.header.frame_id="right_sensor_5_link";
      if(finger==3 && i==5)pressure_marker.header.frame_id="right_sensor_6_link";
      if(finger==3 && i==6)pressure_marker.header.frame_id="right_sensor_7_link";
      if(finger==3 && i==7)pressure_marker.header.frame_id="right_sensor_8_link";

      radius = 0.006;
      height = 0.003;


      pressure_marker.ns = "pressure_markers";      
      pressure_marker.scale.x = radius;
      pressure_marker.scale.y = radius * (4.0 / 5);
      pressure_marker.scale.z = height;

      pressure_marker.pose.position.x=0.012;
      pressure_marker.pose.position.y=0.0;
      pressure_marker.pose.position.z=0.0047;

      pressure_marker.pose.orientation.y=0.707;
      pressure_marker.pose.orientation.w=0.707;
      pressure_val = max(min((pressure_val / -100.0) + 0.8, 1.0), 0.0);
      pressure_marker.color.r = pressure_val;
      pressure_marker.color.g = pressure_val;
      pressure_marker.color.b = pressure_val;
      pressure_marker.color.a = 1;

      contact_marker.id = finger*8+i;
      pressure_marker.id = finger*8+i;

      marker_array.markers.push_back(contact_marker);
      marker_array.markers.push_back(pressure_marker);
    }
  }

  sensor_pub.publish(marker_array);


  joint_state.header.stamp = ros::Time::now();



  //joint_state.header.frame_id="base_link";




  joint_state.name.clear();

              joint_state.position.clear();

              joint_state.velocity.clear();

              joint_state.effort.clear();






  joint_state.name.resize(1);
  joint_state.position.resize(1);

  joint_state.name[0]="right_3_joint";

  up_position = hand->motor_data.position;

  up_position = up_position / 2.00;

  up_position = up_position - 7.235 * 2;
  up_position = up_position / 50.00;

  up_position = acos(up_position) - 0.98; 

  joint_state.position[0] = up_position;

  joint_pub.publish(joint_state);



}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "roshand_visualizer");
    ros::NodeHandle n;

    std::string ns;
    n.getParam("ns", ns);

    joint_pub = n.advertise<sensor_msgs::JointState>(ns + "joint_states", 1);
    sensor_pub = n.advertise<visualization_msgs::MarkerArray>(ns + "visualization_marker_array", 100);


    ros::Subscriber pose_sub = n.subscribe(ns + "/RosHand_data", 1, publish_sensors_to_rviz);

    ros::spin();
    return 0;
}


