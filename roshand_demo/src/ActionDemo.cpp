#include "ActionDemo.h"

namespace roshand_pick_demo
{
bool finished=false;
int count = 0;


roshand_demo::roshand_demo(ros::NodeHandle node)
{

   client_without_sensor = new actionlib::SimpleActionClient<roshand_msg::CommandMotorWithoutSensorAction>
            ( "/roshand_bringup/RosHandMove_WithOut_Sensor", false);
    while( !client_without_sensor->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the finger action without sensor server to come up");
    }

    client_with_sensor = new actionlib::SimpleActionClient<roshand_msg::CommandMotorWithSensorAction>
            ("/roshand_bringup/RosHandMove_With_Sensor", false);
    while( !client_with_sensor->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the finger action with sensor server to come up");
    }
//open 
    ROS_INFO("Start open without sensor.....");
    finished = false;
    while(finished == false)
    {
        finished = grasp_useless_sensor(130000);
    }

    ros::WallDuration(2).sleep();
// close
    ROS_INFO("Start close without sensor.....");
    finished = false;
    while(finished == false)
    {
        finished = grasp_useless_sensor(1000);
    }

   ros::WallDuration(2).sleep();


//open
    ROS_INFO("Start open without sensor.....");
    finished = false;
    while(finished == false)
    {
        finished = grasp_useless_sensor(130000);
    }


    ros::WallDuration(2).sleep();
    ROS_INFO("Start close with sensor.....");
    finished = false;
    while(finished == false)
    {
        finished = touch_grasp((uint8_t)16, (uint8_t)1);
    }




    ros::WallDuration(2).sleep();
    ROS_INFO("Start open without sensor.....");
    finished = false;
    while(finished == false)
    {
        finished = grasp_useless_sensor(130000);
    }

}
roshand_demo::~roshand_demo()
{

    delete client_without_sensor;
    delete client_with_sensor;

}



bool roshand_demo::touch_grasp(uint8_t close_step, uint8_t open_step)
{

        roshand_msg::CommandMotorWithSensorGoal demo_goal;

        
        demo_goal.close_step_mag = close_step;  
        demo_goal.open_step_masg = open_step;  

        client_with_sensor -> sendGoal(demo_goal);

        if (client_with_sensor -> waitForResult(ros::Duration(5.0)))
        {
            client_with_sensor -> getResult();
            return true;
        }
        else
        {
            client_with_sensor -> cancelAllGoals();
            ROS_WARN_STREAM("The gripper action with sensor timed-out");
            return false;
        }



}

bool roshand_demo::grasp_useless_sensor(uint32_t motor_turn)
{

        roshand_msg::CommandMotorWithoutSensorGoal demo_goal;

        
        demo_goal.goal_position = motor_turn;  

        client_without_sensor -> sendGoal(demo_goal);

        if (client_without_sensor -> waitForResult(ros::Duration(5.0)))
        {
            client_without_sensor -> getResult();
            return true;
        }
        else
        {
            client_without_sensor -> cancelAllGoals();
            ROS_WARN_STREAM("The gripper action without sensor timed-out");
            return false;
        }



};
}

int main(int argc, char **argv)
{

        ros::init(argc, argv, "action_client_demo");

        ros::NodeHandle node;

       
        ros::AsyncSpinner spinner(2);
        spinner.start();
        roshand_pick_demo::roshand_demo demo(node);

        //ros::waitForShutdown();
        //ros::spin();
        return 0;
}
