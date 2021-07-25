#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include "roshand_msg/CommandMotorWithoutSensorAction.h"    
#include "roshand_msg/CommandMotorWithSensorAction.h"  

namespace roshand_pick_demo
{

  class roshand_demo
  {
      public:
        roshand_demo(ros::NodeHandle node);
        ~roshand_demo();

      private:
          actionlib::SimpleActionClient<roshand_msg::CommandMotorWithoutSensorAction>* client_without_sensor;
          actionlib::SimpleActionClient<roshand_msg::CommandMotorWithSensorAction>* client_with_sensor;
          
          bool grasp_useless_sensor(uint32_t motor_turn);
          bool touch_grasp(uint8_t close_step, uint8_t open_step);


  };

}  


