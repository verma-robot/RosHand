#ifndef ROSHAND_H
#define ROSHAND_H

#include <ros/ros.h>
#include <ros/time.h>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <iostream>
#include <fstream>
#include <sstream>

#include "roshand_msg/Hand.h"
#include "roshand_msg/FingerPressure.h"
#include "roshand_msg/Motor.h"
#include "roshand_msg/HandCommandWithoutSensor.h"
#include "roshand_msg/SensorCalibrate.h"
//#include "roshand_msg/CommandUseSensor.h"
#include "roshand_msg/HandCommandWithSensor.h"
#include "roshand_msg/DisableSensor.h"
#include "roshand_msg/EnableSensor.h"
#include "roshand_msg/CommandMotorWithoutSensorAction.h"    
#include "roshand_msg/CommandMotorWithSensorAction.h"  

#include <std_srvs/Empty.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <yaml-cpp/yaml.h>

#include <actionlib/server/simple_action_server.h>
#include "sensor_msgs/JointState.h"

namespace roshand
{     
   
       typedef union
       {

          signed char u[2];
          int16_t f;

       }sensor_value;

       typedef union
       {

          signed char u[4];
          int32_t f;

       }motor_position;

       typedef union
       {

          signed char u[2];
          int16_t f;

       }motor_speed;

       class roshand_hardware
       {
	    public:


                    roshand_msg::Hand hand_data;

                    bool CALIBRATE_SENSOR_FINESHED = false;
                    bool CLOSE_WITH_SENSOR_COMMAND = false;
                    bool MOVE_WITHOUT_SENSOR = false;
                    bool OPEN_WITH_SENSOR_COMMAND = false;
                    bool SET_SENSOR_BIAS = false;
                    bool SET_SENSOR_THREADHOLD = false;
                    bool READ_DATA = false;

                    std::vector<int> sensor_thread_hold[4];
                    bool control_finished = true;
                   // sensor_msgs::JointState joint_state;

            public:

		    roshand_hardware(void);
                    ~roshand_hardware();   
       		    bool init(std:: string port_name, int port_rate);

                    void handle_read( char *buf, boost::system::error_code ec, std::size_t bytes_transferred );

                    void read_data();
                    //void calibrate_sensor();

                    void set_position_without_sensor(uint32_t target_position);
                    void close_with_sensor(uint8_t close_step_mag, uint8_t open_step_mag);
                    void open_with_sensor();
                    void set_sensor_thread_hold();

                    void set_sensor_bias(int sensor_hi);

                    //bool use_sensor_enable();

                    void listen_data(uint8_t data_number, int max_seconds);
                    void calibrate_data();

                    //bool setThreadHold();
                    bool spOnce(uint32_t target_position, bool use_sensor, bool use_without_action, bool control_finished ,uint8_t close_step, uint8_t open_step);



            private:
                  
                   // bool happy_;

              
		    //ros::NodeHandle nh;
		    //ros::Publisher pub;  



                    ros::Time current_time, last_time;


                    boost::asio::serial_port *sp;
                    boost::asio::io_service iosev;
                    boost::system::error_code ec;

                    std::string port_name; 
                    int port_rate;
                    //std::string dir; 
                    //int sensor_bias;


                    int READ_BUFFER_SIZE;
                    //bool READ_BUFFER_FINISHED=false;

                    //bool OPEN_FINISHED_FLAG=false;


                    int16_t sensor_Max_value = 255;   
                    std::vector <int> second_finger_calibrate_data;
                    std::vector <int> third_finger_calibrate_data;
                    std::vector <int> palm_calibrate_data;


        };


    
}

#endif /* SENSOR_H */
