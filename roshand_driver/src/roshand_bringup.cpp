#include "roshand_driver.h"

uint32_t target_poition;
bool close_hand_with_sensor=false;
bool open_hand_with_sensor=false;
bool ready;
uint32_t max_position = 130000;
uint8_t max_close_step_mag = 16;
uint8_t max_open_setp_mag = 4;

uint8_t close_step_mag;
uint8_t open_step_mag;

bool use_without_action ;

bool use_sensor;
roshand::roshand_hardware hand;	
ros::Publisher pub;
ros::Publisher pub_joint_state;

roshand_msg::DisableSensor	sensor_disabled;
roshand_msg::EnableSensor	sensor_enable;				

typedef actionlib::SimpleActionServer<roshand_msg::CommandMotorWithoutSensorAction> Server_Close_WithOut_Sensor;
typedef actionlib::SimpleActionServer<roshand_msg::CommandMotorWithSensorAction> Server_Close_With_Sensor;

// move without sensor
void execute_without_sensor(const roshand_msg::CommandMotorWithoutSensorGoalConstPtr& demo_goal, Server_Close_WithOut_Sensor* as)
{
    ros::Rate r(10); 
    roshand_msg::CommandMotorWithoutSensorFeedback feedback;   
    roshand_msg::CommandMotorWithoutSensorResult result_;    /* 创建一个feedback对象 */    
    
    int count = 0;
    while(use_without_action == true)use_without_action = false;
    //loop 
    while(hand.MOVE_WITHOUT_SENSOR == true)hand.MOVE_WITHOUT_SENSOR = false;
    while(hand.MOVE_WITHOUT_SENSOR == false)
    {

        if((demo_goal -> goal_position) > max_position)hand.set_position_without_sensor(max_position);
        else hand.set_position_without_sensor(demo_goal -> goal_position);
 

        feedback.state = hand.hand_data;
        as ->  publishFeedback(feedback);
        count += 1;

        r.sleep();
    }

    while(result_.finish == false)result_.finish = hand.MOVE_WITHOUT_SENSOR;
    as -> setSucceeded(result_); 
    while(hand.MOVE_WITHOUT_SENSOR == true)hand.MOVE_WITHOUT_SENSOR = false;

}



void execute_with_sensor(const roshand_msg::CommandMotorWithSensorGoalConstPtr& demo_goal, Server_Close_With_Sensor* as)
{
    ros::Rate r(10); 
    roshand_msg::CommandMotorWithSensorFeedback feedback;    /* 创建一个feedback对象 */    
    roshand_msg::CommandMotorWithSensorResult result_;    /* result */    

    int count = 0;

    while(use_without_action == true)use_without_action = false;
    while(hand.CLOSE_WITH_SENSOR_COMMAND == true)hand.CLOSE_WITH_SENSOR_COMMAND = false;
    while(hand.CLOSE_WITH_SENSOR_COMMAND == false )
    {

        if((uint8_t)(demo_goal -> close_step_mag) > max_close_step_mag && (uint8_t)(demo_goal -> open_step_masg) < max_open_setp_mag)hand.close_with_sensor(max_close_step_mag, ((uint8_t)(demo_goal -> open_step_masg)));
        else if((uint8_t)(demo_goal -> close_step_mag) > max_close_step_mag && (uint8_t)(demo_goal -> open_step_masg) > max_open_setp_mag)    hand.close_with_sensor(max_close_step_mag, max_open_setp_mag);
        else if((uint8_t)(demo_goal -> close_step_mag) < max_close_step_mag && (uint8_t)(demo_goal -> open_step_masg) > max_open_setp_mag)    hand.close_with_sensor(((uint8_t)(demo_goal -> close_step_mag)), max_open_setp_mag);

        else hand.close_with_sensor(((uint8_t)(demo_goal -> close_step_mag)), ((uint8_t)(demo_goal -> open_step_masg))); 

        feedback.state = hand.hand_data;
        as ->  publishFeedback(feedback);

        r.sleep();
        count += 1;
   }


    while(result_.finish == false)result_.finish = hand.CLOSE_WITH_SENSOR_COMMAND;
    as->setSucceeded(result_); 
    while(hand.CLOSE_WITH_SENSOR_COMMAND == true)hand.CLOSE_WITH_SENSOR_COMMAND = false;    

}

bool calibrate_senser( roshand_msg::SensorCalibrate::Request& req,  roshand_msg::SensorCalibrate::Response& res)
{   

   while(hand.CALIBRATE_SENSOR_FINESHED == false)hand.calibrate_data();
   if(hand.CALIBRATE_SENSOR_FINESHED == true){
        ROS_INFO("SENSOR CALIBRATE FINISHED............"); 
        while(res.CALIBRATE_FINISHED == false)res.CALIBRATE_FINISHED = hand.CALIBRATE_SENSOR_FINESHED ;
   }
   else {
        while(res.CALIBRATE_FINISHED == true)hand.CALIBRATE_SENSOR_FINESHED = false;
        while(res.CALIBRATE_FINISHED == true)res.CALIBRATE_FINISHED = hand.CALIBRATE_SENSOR_FINESHED ;
   }
   while(hand.CALIBRATE_SENSOR_FINESHED == true)hand.CALIBRATE_SENSOR_FINESHED=false;
   return true;

}


bool set_target_position( roshand_msg::HandCommandWithoutSensor::Request &req,  roshand_msg::HandCommandWithoutSensor::Response &res)
{

     target_poition = req.target_turn;
     //std::cout << target_poition <<std::endl;

     while(use_without_action == false)use_without_action = true;

     if(use_sensor == false){

        if(hand.MOVE_WITHOUT_SENSOR == true){

             while(hand.MOVE_WITHOUT_SENSOR == true)hand.MOVE_WITHOUT_SENSOR=false;
             while(res.Command_Send == false)res.Command_Send = true;
             while(hand.control_finished == false)hand.control_finished = true;
        }
        else { while(res.Command_Send == true)res.Command_Send = false; while(hand.control_finished == true)hand.control_finished = false;}
     }
     else {  

        while(res.Command_Send == true)res.Command_Send = false;
        while(hand.CLOSE_WITH_SENSOR_COMMAND == true)hand.MOVE_WITHOUT_SENSOR = false;
        target_poition = max_position;



     }
     return true;

}



bool enable_sensor_touch_service( roshand_msg::EnableSensor::Request &req,  roshand_msg::EnableSensor::Response &res)
{


   while(use_sensor == false)use_sensor = true;

   if(use_sensor == true){while(res.SENSOR_ENABLED == false)res.SENSOR_ENABLED = true; ROS_INFO("SENSOR ENABLED  WHEN GRASP............"); }
   else {while(res.SENSOR_ENABLED == true)res.SENSOR_ENABLED = false;}


  // std::cout << "use sensor" << std::endl;

   return true;

}


bool disable_sensor_touch_service( roshand_msg::DisableSensor::Request &req,  roshand_msg::DisableSensor::Response &res)
{



   while(use_sensor == true)use_sensor = false;//not use sensor
   if(use_sensor == false){while(res.SENSOR_DISABLED == false)res.SENSOR_DISABLED = true; ROS_INFO("SENSOR DISABLED  WHEN GRASP............");}
   else {while(res.SENSOR_DISABLED == true)res.SENSOR_DISABLED = false;}
   //std::cout << "don't use sensor" << std::endl;
   //use_without_action = true;
   return true;

}



bool close_hand_with_senser( roshand_msg::HandCommandWithSensor::Request &req,  roshand_msg::HandCommandWithSensor::Response &res)
{
   if(use_sensor == true)
   {

      close_step_mag = req.close_step_mag;
      open_step_mag = req.open_step_mag;
      while(use_without_action == false)use_without_action = true;
      if(close_step_mag > max_close_step_mag)close_step_mag = max_close_step_mag;
      if(open_step_mag > max_open_setp_mag)close_step_mag = max_open_setp_mag;

      if(hand.CLOSE_WITH_SENSOR_COMMAND == true)
      {
           while(res.Command_Send == false)res.Command_Send = true; 
           while(hand.CLOSE_WITH_SENSOR_COMMAND == true)hand.CLOSE_WITH_SENSOR_COMMAND = false;
           while(hand.control_finished == false)hand.control_finished = true;
      }
      else {
           while(res.Command_Send == true)res.Command_Send = false; 
           while(hand.control_finished == true)hand.control_finished = false;
      }
   }
   else {

      while(res.Command_Send == true)res.Command_Send = false;
      while(hand.CLOSE_WITH_SENSOR_COMMAND == true)hand.CLOSE_WITH_SENSOR_COMMAND = false;
      close_step_mag = close_step_mag;
      open_step_mag = open_step_mag;

   }
  
  

   //use_without_action = true;
   return true;

}



int main(int argc, char** argv)
{

	ros::init(argc, argv, "roshand_driver_node");	
	
        ros::NodeHandle nh("~");


        std::string port_name; 
        int port_rate;
        std::string dir; 
        int sensor_bias;
        std::string ns;

        std::vector<int> all_sensor_thread_hold[4];

        nh.getParam("port_name", port_name);
        nh.getParam("port_rate", port_rate);

        nh.getParam("sensor_bias", sensor_bias);

        nh.getParam("left_fingertips_thread_hold", all_sensor_thread_hold[0]);
        nh.getParam("left_linear_finger_thread_hold", all_sensor_thread_hold[1]);
        nh.getParam("right_fingertips_thread_hold", all_sensor_thread_hold[2]);
        nh.getParam("right_linear_finger_thread_hold", all_sensor_thread_hold[3]);

        nh.getParam("ns", ns);
	 
        hand.init(port_name, port_rate);


        while(hand.SET_SENSOR_BIAS == false)hand.set_sensor_bias(sensor_bias);
        ROS_INFO("SENSOR BIAS SET SUCCEED....................");


        hand.sensor_thread_hold[0]=all_sensor_thread_hold[0];
        hand.sensor_thread_hold[1]=all_sensor_thread_hold[1];
        hand.sensor_thread_hold[2]=all_sensor_thread_hold[2];
        hand.sensor_thread_hold[3]=all_sensor_thread_hold[3];

        while(hand.SET_SENSOR_THREADHOLD == false)hand.set_sensor_thread_hold();
        ROS_INFO("SENSOR THREADHOLD SET SUCCEED....................");

        ros::ServiceServer calibrate_sensor = nh.advertiseService(ns + "/calibrate_sensor_service", calibrate_senser);	

        ros::ServiceServer close_hand_with_sensor_service = nh.advertiseService(ns + "/close_hand_with_sensor_service", close_hand_with_senser);
       
        ros::ServiceServer disable_sensor_touch = nh.advertiseService(ns + "/disable_sensor_touch_service", disable_sensor_touch_service);
        ros::ServiceServer enable_sensor_touch = nh.advertiseService(ns + "/enable_sensor_touch_service", enable_sensor_touch_service);

        ros::ServiceServer set_motor_target_position = nh.advertiseService(ns + "/set_motor_target_position", set_target_position);


        pub = nh.advertise<roshand_msg::Hand>(ns + "/RosHand_data", 50);
       // pub_joint_state = nh.advertise<sensor_msgs::JointState>(ns + "/joint_states", 50);

        Server_Close_WithOut_Sensor server_without_sensor(nh, "RosHandMove_WithOut_Sensor", boost::bind(&execute_without_sensor, _1, &server_without_sensor), false);
        server_without_sensor.start();


        Server_Close_With_Sensor server_with_sensor(nh, "RosHandMove_With_Sensor", boost::bind(&execute_with_sensor, _1, &server_with_sensor), false);
        server_with_sensor.start();

        use_sensor=true;
        hand.control_finished = true;
        use_without_action = false;

	ros::Rate loop_rate(50);

	while (ros::ok()) 
	{

            try
            {

               hand.read_data();

               pub.publish(hand.hand_data); 
               //pub_joint_state.publish(hand.joint_state);


               ready = hand.spOnce(target_poition, use_sensor, use_without_action, hand.control_finished, close_step_mag, open_step_mag);
//touch sensor
               if(ready && use_sensor == true && hand.CLOSE_WITH_SENSOR_COMMAND == true && use_without_action == true)hand.control_finished = true;
               else if(ready && use_sensor == true && hand.CLOSE_WITH_SENSOR_COMMAND == false && use_without_action == true)hand.control_finished = false;

//without sensor
               else if(ready && use_sensor == false && hand.MOVE_WITHOUT_SENSOR == false && use_without_action == true)hand.control_finished = false;

               else if(ready && use_sensor == false && hand.MOVE_WITHOUT_SENSOR == true && use_without_action == true)hand.control_finished = true;
	      if(use_without_action == false)hand.control_finished = true;

               ros::spinOnce();
	       loop_rate.sleep();
 


            }
            catch(const std::exception& e)
            {
		ROS_ERROR("//////////////ERROR/////////////.");


            }

	}

	return 0;
}


