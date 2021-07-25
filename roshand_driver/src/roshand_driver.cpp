#include <vector>
#include "roshand_driver.h"

namespace roshand {


   roshand_hardware::roshand_hardware(){ }

   roshand_hardware::~roshand_hardware() {
     if(sp)delete sp;

     if (sp) {
     sp -> cancel();
     sp -> close();

     }

     iosev.stop();
     iosev.reset();

     delete sp;
  }


  bool roshand_hardware::init(std:: string port_name, int port_rate) {


         sp = new boost::asio::serial_port(iosev);

         try {

              sp -> open(port_name, ec); 
              sp -> set_option(boost::asio::serial_port::baud_rate(port_rate));
	      sp -> set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
	      sp -> set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
	      sp -> set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
	      sp -> set_option(boost::asio::serial_port::character_size(8));

              ros::Time::init();
	      current_time = ros::Time::now();
	      last_time = ros::Time::now();

              //set_sensor_bias(sensor_bias);

         }
         catch(...) {

              ROS_ERROR( "Can't open serial port") ;
         }

         return true;
  }

void roshand_hardware::set_sensor_thread_hold( void ) {

    uint8_t data_read[37] = {0xFF, 0x01, 0x04, 0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x6D};  

    data_read[4] = sensor_thread_hold[0][0];
    data_read[5] = sensor_thread_hold[0][1];
    data_read[6] = sensor_thread_hold[0][2];
    data_read[7] = sensor_thread_hold[0][3];
    data_read[8] = sensor_thread_hold[0][4];
    data_read[9] = sensor_thread_hold[0][5];
    data_read[10] = sensor_thread_hold[0][6];
    data_read[11] = sensor_thread_hold[0][7];


    data_read[12]= sensor_thread_hold[1][0];
    data_read[13]= sensor_thread_hold[1][1];
    data_read[14]= sensor_thread_hold[1][2];
    data_read[15]= sensor_thread_hold[1][3];
    data_read[16]= sensor_thread_hold[1][4];
    data_read[17]= sensor_thread_hold[1][5];
    data_read[18]= sensor_thread_hold[1][6];
    data_read[19]= sensor_thread_hold[1][7];

    data_read[20] = sensor_thread_hold[2][0];
    data_read[21] = sensor_thread_hold[2][1];
    data_read[22] = sensor_thread_hold[2][2];
    data_read[23] = sensor_thread_hold[2][3];
    data_read[24] = sensor_thread_hold[2][4];
    data_read[25] = sensor_thread_hold[2][5];
    data_read[26] = sensor_thread_hold[2][6];
    data_read[27] = sensor_thread_hold[2][7];

    data_read[28] = sensor_thread_hold[3][0];
    data_read[29] = sensor_thread_hold[3][1];
    data_read[30] = sensor_thread_hold[3][2];
    data_read[31] = sensor_thread_hold[3][3];
    data_read[32] = sensor_thread_hold[3][4];
    data_read[33] = sensor_thread_hold[3][5];
    data_read[34] = sensor_thread_hold[3][6];
    data_read[35] = sensor_thread_hold[3][7];

    boost::asio::write(*sp, boost::asio::buffer(&data_read[0], 37), ec);

    listen_data(37, 20);
}


void roshand_hardware::handle_read( char buf[], boost::system::error_code ec, std::size_t bytes_transferred )
{


    READ_BUFFER_SIZE = bytes_transferred;

    if(READ_BUFFER_SIZE ==5)
    {   

         if(buf[2] == 0x0A && buf[3] == 0x00 && buf[4] == 0x6D)while(CALIBRATE_SENSOR_FINESHED == false)CALIBRATE_SENSOR_FINESHED = true; //calibrate sensor succeed
         else if(buf[2]==0x08 && buf[3]==0x00 && buf[4]==0x6D){while(CLOSE_WITH_SENSOR_COMMAND == true)CLOSE_WITH_SENSOR_COMMAND = false; }
         else if(buf[2]==0x08 && buf[3]==0x01 && buf[4]==0x6D){while(CLOSE_WITH_SENSOR_COMMAND == false)CLOSE_WITH_SENSOR_COMMAND = true; }

         else if(buf[2]==0x09 && buf[3]==0x00 && buf[4]==0x6D){while(OPEN_WITH_SENSOR_COMMAND == true)OPEN_WITH_SENSOR_COMMAND = false; }
         else if(buf[2]==0x09 && buf[3]==0x01 && buf[4]==0x6D){while(OPEN_WITH_SENSOR_COMMAND == false)OPEN_WITH_SENSOR_COMMAND = true; }

         else if(buf[2]==0x07 && buf[3]==0x00 && buf[4]==0x6D){while(MOVE_WITHOUT_SENSOR == true)MOVE_WITHOUT_SENSOR = false; }
         else if(buf[2]==0x07 && buf[3]==0x01 && buf[4]==0x6D){while(MOVE_WITHOUT_SENSOR == false)MOVE_WITHOUT_SENSOR = true; }
         else if(buf[2]==0x05 && buf[3]==0x00 && buf[4]==0x6D){while(SET_SENSOR_BIAS == false)SET_SENSOR_BIAS = true; }//set sensor bias succeed
         for(int i = 0 ; i < 5 ; i++ )buf[i] = 0;//clear the buf

    }
    else if(READ_BUFFER_SIZE == 8)
    {

          //if(buf[2]==0x82 && buf[3]==0x03 && buf[7]==0x6D)MOTOR_DISCONECT=true;
              for(int i = 0 ; i < 8 ; i++ )buf[i] = 0;

    }
    else if(READ_BUFFER_SIZE == 37)
    {
           //std::cout <<  READ_BUFFER_SIZE<<std::endl ;
           if(buf[2] == 0x04 && buf[3] == 0x20 && buf[36] == 0x6D)
           {


                 sensor_thread_hold[0][0] = buf[4 + 0 * 8 + 0];
                 sensor_thread_hold[0][1] = buf[4 + 0 * 8 + 1];
                 sensor_thread_hold[0][2] = buf[4 + 0 * 8 + 2];
                 sensor_thread_hold[0][3] = buf[4 + 0 * 8 + 3];
                 sensor_thread_hold[0][4] = buf[4 + 0 * 8 + 4];
                 sensor_thread_hold[0][5] = buf[4 + 0 * 8 + 5];
                 sensor_thread_hold[0][6] = buf[4 + 0 * 8 + 6];
                 sensor_thread_hold[0][7] = buf[4 + 0 * 8 + 7];


                 sensor_thread_hold[1][0] = buf[4 + 1 * 8 + 0];
                 sensor_thread_hold[1][1] = buf[4 + 1 * 8 + 1];
                 sensor_thread_hold[1][2] = buf[4 + 1 * 8 + 2];
                 sensor_thread_hold[1][3] = buf[4 + 1 * 8 + 3];
                 sensor_thread_hold[1][4] = buf[4 + 1 * 8 + 4];
                 sensor_thread_hold[1][5] = buf[4 + 1 * 8 + 5];
                 sensor_thread_hold[1][6] = buf[4 + 1 * 8 + 6 ];
                 sensor_thread_hold[1][7] = buf[4 + 1 * 8 + 7];

                 sensor_thread_hold[2][0] = buf[4 + 2 * 8 + 0];
                 sensor_thread_hold[2][1] = buf[4 + 2 * 8 + 1];
                 sensor_thread_hold[2][2] = buf[4 + 2 * 8 + 2];
                 sensor_thread_hold[2][3] = buf[4 + 2 * 8 + 3];
                 sensor_thread_hold[2][4] = buf[4 + 2 * 8 + 4];
                 sensor_thread_hold[2][5] = buf[4 + 2 * 8 + 5];
                 sensor_thread_hold[2][6] = buf[4 + 2 * 8 + 6];
                 sensor_thread_hold[2][7] = buf[4 + 2 * 8 + 7];

                 sensor_thread_hold[3][0] = buf[4 + 3 * 8 + 0];
                 sensor_thread_hold[3][1] = buf[4 + 3 * 8 + 1];
                 sensor_thread_hold[3][2] = buf[4 + 3 * 8 + 2];
                 sensor_thread_hold[3][3] = buf[4 + 3 * 8 + 3];
                 sensor_thread_hold[3][4] = buf[4 + 3 * 8 + 4];
                 sensor_thread_hold[3][5] = buf[4 + 3 * 8 + 5];
                 sensor_thread_hold[3][6] = buf[4 + 3 * 8 + 6];
                 sensor_thread_hold[3][7] = buf[4 + 3 * 8 + 7];
                 while(SET_SENSOR_THREADHOLD == false)SET_SENSOR_THREADHOLD = true;//set sensor threadhold succeed

           }
           for(int i = 0 ; i < 37 ; i++ )buf[i] = 0;//clear buf

    }
    else if (READ_BUFFER_SIZE == 112)
    {


           if(buf[2] == 0x03 && buf[3] == 0x6B && buf[111] == 0x6D)
           {

              uint8_t i,j;
              uint32_t position;
              uint16_t speed;
              uint16_t sensor[8];
              bool contact[8];
              double y_, m_, n_, b_;
              sensor_value data_raw;
              motor_position motor_position;
              motor_speed    motor_speed;



              data_raw.u[1] = buf[4 + 0 * 16 + 0 * 2];
              data_raw.u[0] = buf[5 + 0 * 16 + 0 * 2];
              hand_data.finger[0].sensor[0] = data_raw.f;

              //if(data_raw.f > sensor_Max_value)hand_data.finger[0].sensor[0]=sensor_Max_value;
              hand_data.finger[0].contact[0] = (bool)((uint8_t)buf[68 + 0 * 8 + 0]);   
              hand_data.finger[0].threadhold[0] = (uint8_t)sensor_thread_hold[0][0];   


              data_raw.u[1] = buf[4 + 0 * 16 + 1 * 2];
              data_raw.u[0] = buf[5 + 0 * 16 + 1 * 2];
              hand_data.finger[0].sensor[1] = data_raw.f;
              //if(data_raw.f>sensor_Max_value)hand_data.finger[0].sensor[1]=sensor_Max_value;
              hand_data.finger[0].contact[1] = (bool)((uint8_t)buf[68 + 0 * 8 + 1]);  
              hand_data.finger[0].threadhold[1] = (uint8_t)sensor_thread_hold[0][1];   



              data_raw.u[1] = buf[4 + 0 * 16 + 2 * 2];
              data_raw.u[0] = buf[5 + 0 * 16 + 2 * 2];
              hand_data.finger[0].sensor[2] = data_raw.f;
              //if(data_raw.f>sensor_Max_value)hand_data.finger[0].sensor[2]=sensor_Max_value;
              hand_data.finger[0].contact[2] = (bool)((uint8_t)buf[68 + 0 * 8 + 2]); 
              hand_data.finger[0].threadhold[2] = (uint8_t)sensor_thread_hold[0][2];    


              data_raw.u[1] = buf[4 + 0 * 16 + 3 * 2];
              data_raw.u[0] = buf[5 + 0 * 16 + 3 * 2];
              hand_data.finger[0].sensor[3] = data_raw.f;
              //if(data_raw.f>sensor_Max_value)hand_data.finger[0].sensor[3]=sensor_Max_value;
              hand_data.finger[0].contact[3] = (bool)((uint8_t)buf[68 + 0 * 8 + 3]);  
              hand_data.finger[0].threadhold[3] = (uint8_t)sensor_thread_hold[0][3];    

              data_raw.u[1] = buf[4 + 0 * 16 + 4 * 2];
              data_raw.u[0] = buf[5 + 0 * 16 + 4 * 2];
              hand_data.finger[0].sensor[4] = data_raw.f;
              //if(data_raw.f>sensor_Max_value)hand_data.finger[0].sensor[4]=sensor_Max_value;
              hand_data.finger[0].contact[4] = (bool)((uint8_t)buf[68 + 0 * 8 + 4]);  
              hand_data.finger[0].threadhold[4] = (uint8_t)sensor_thread_hold[0][4];    

              data_raw.u[1] = buf[4 + 0 * 16 + 5 * 2];
              data_raw.u[0] = buf[5 + 0 * 16 + 5 * 2];
              hand_data.finger[0].sensor[5] = data_raw.f;
             // if(data_raw.f>sensor_Max_value)hand_data.finger[0].sensor[5]=sensor_Max_value;
              hand_data.finger[0].contact[5] = (bool)((uint8_t)buf[68 + 0 * 8 + 5]);  
              hand_data.finger[0].threadhold[5] = (uint8_t)sensor_thread_hold[0][5];    


              data_raw.u[1] = buf[4 + 0 * 16 + 6 * 2];
              data_raw.u[0] = buf[5 + 0 * 16 + 6 * 2];
              hand_data.finger[0].sensor[6] = data_raw.f;
              //if(data_raw.f>sensor_Max_value)hand_data.finger[0].sensor[6]=sensor_Max_value;
              hand_data.finger[0].contact[6] = (bool)((uint8_t)buf[68 + 0 * 8 + 6]); 
              hand_data.finger[0].threadhold[6] = (uint8_t)sensor_thread_hold[0][6];     

              data_raw.u[1] = buf[4 + 0 * 16 + 7 * 2];
              data_raw.u[0] = buf[5 + 0 * 16 + 7 * 2];
              hand_data.finger[0].sensor[7] = data_raw.f;
              //if(data_raw.f>sensor_Max_value)hand_data.finger[0].sensor[7]=sensor_Max_value;
              hand_data.finger[0].contact[7] = (bool)((uint8_t)buf[68 + 0 * 8 + 7]);  
              hand_data.finger[0].threadhold[7] = (uint8_t)sensor_thread_hold[0][7]; 


              data_raw.u[1] = buf[4 + 1 * 16 + 0 * 2];
              data_raw.u[0] = buf[5 + 1 * 16 + 0*2];
              hand_data.finger[1].sensor[0] = data_raw.f;
             // if(data_raw.f>sensor_Max_value)hand_data.finger[1].sensor[0]=sensor_Max_value;
              hand_data.finger[1].contact[0] = (bool)((uint8_t)buf[68 + 1 * 8 + 0]); 
              hand_data.finger[1].threadhold[0] = (uint8_t)sensor_thread_hold[1][0];      

              data_raw.u[1] = buf[4 + 1 * 16 + 1 * 2];
              data_raw.u[0] = buf[5 + 1 * 16 + 1 * 2];
              hand_data.finger[1].sensor[1] = data_raw.f;
             // if(data_raw.f>sensor_Max_value)hand_data.finger[1].sensor[1]=sensor_Max_value;
              hand_data.finger[1].contact[1] = (bool)((uint8_t)buf[68 + 1 * 8 + 1]); 
              hand_data.finger[1].threadhold[1] = (uint8_t)sensor_thread_hold[1][1]; 

              data_raw.u[1]=buf[4+1*16+2*2];
              data_raw.u[0]=buf[5+1*16+2*2];
              hand_data.finger[1].sensor[2]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[1].sensor[2]=sensor_Max_value;
              hand_data.finger[1].contact[2]=(bool)((uint8_t)buf[68+1*8+2]);  
              hand_data.finger[1].threadhold[2]=(uint8_t)sensor_thread_hold[1][2];    


              data_raw.u[1]=buf[4+1*16+3*2];
              data_raw.u[0]=buf[5+1*16+3*2];
              hand_data.finger[1].sensor[3]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[1].sensor[3]=sensor_Max_value;
              hand_data.finger[1].contact[3]=(bool)((uint8_t)buf[68+1*8+3]); 
              hand_data.finger[1].threadhold[3]=(uint8_t)sensor_thread_hold[1][3];     


              data_raw.u[1]=buf[4+1*16+4*2];
              data_raw.u[0]=buf[5+1*16+4*2];
              hand_data.finger[1].sensor[4]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[1].sensor[4]=sensor_Max_value;
              hand_data.finger[1].contact[4]=(bool)((uint8_t)buf[68+1*8+4]);  
              hand_data.finger[1].threadhold[4]=(uint8_t)sensor_thread_hold[1][4];    

              data_raw.u[1]=buf[4+1*16+5*2];
              data_raw.u[0]=buf[5+1*16+5*2];
              hand_data.finger[1].sensor[5]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[1].sensor[5]=sensor_Max_value;
              hand_data.finger[1].contact[5]=(bool)((uint8_t)buf[68+1*8+5]);
              hand_data.finger[1].threadhold[5]=(uint8_t)sensor_thread_hold[1][5];      


              data_raw.u[1]=buf[4+1*16+6*2];
              data_raw.u[0]=buf[5+1*16+6*2];
              hand_data.finger[1].sensor[6]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[1].sensor[6]=sensor_Max_value;
              hand_data.finger[1].contact[6]=(bool)((uint8_t)buf[68+1*8+6]);  
              hand_data.finger[1].threadhold[6]=(uint8_t)sensor_thread_hold[1][6];    

              data_raw.u[1]=buf[4+1*16+7*2];
              data_raw.u[0]=buf[5+1*16+7*2];
              hand_data.finger[1].sensor[7]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[1].sensor[7]=sensor_Max_value;
              hand_data.finger[1].contact[7]=(bool)((uint8_t)buf[68+1*8+7]);                
              hand_data.finger[1].threadhold[7]=(uint8_t)sensor_thread_hold[1][7];    

              data_raw.u[1]=buf[4+2*16+0*2];
              data_raw.u[0]=buf[5+2*16+0*2];
              hand_data.finger[2].sensor[0]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[2].sensor[0]=sensor_Max_value;
              hand_data.finger[2].contact[0]=(bool)((uint8_t)buf[68+2*8+0]);  
              hand_data.finger[2].threadhold[0]=(uint8_t)sensor_thread_hold[2][0];     
                  // finger.contact[jj]=true;

              data_raw.u[1]=buf[4+2*16+1*2];
              data_raw.u[0]=buf[5+2*16+1*2];
              hand_data.finger[2].sensor[1]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[2].sensor[1]=sensor_Max_value;
              hand_data.finger[2].contact[1]=(bool)((uint8_t)buf[68+2*8+1]);  
              hand_data.finger[2].threadhold[1]=(uint8_t)sensor_thread_hold[2][1];    

              data_raw.u[1]=buf[4+2*16+2*2];
              data_raw.u[0]=buf[5+2*16+2*2];
              hand_data.finger[2].sensor[2]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[2].sensor[2]=sensor_Max_value;
              hand_data.finger[2].contact[2]=(bool)((uint8_t)buf[68+2*8+2]); 
              hand_data.finger[2].threadhold[2]=(uint8_t)sensor_thread_hold[2][2];      


              data_raw.u[1]=buf[4+2*16+3*2];
              data_raw.u[0]=buf[5+2*16+3*2];
              hand_data.finger[2].sensor[3]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[2].sensor[3]=sensor_Max_value;
              hand_data.finger[2].contact[3]=(bool)((uint8_t)buf[68+2*8+3]); 
              hand_data.finger[2].threadhold[3]=(uint8_t)sensor_thread_hold[2][3];     


              data_raw.u[1]=buf[4+2*16+4*2];
              data_raw.u[0]=buf[5+2*16+4*2];
              hand_data.finger[2].sensor[4]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[2].sensor[4]=sensor_Max_value;
              hand_data.finger[2].contact[4]=(bool)((uint8_t)buf[68+2*8+4]); 
              hand_data.finger[2].threadhold[4]=(uint8_t)sensor_thread_hold[2][4];      

              data_raw.u[1]=buf[4+2*16+5*2];
              data_raw.u[0]=buf[5+2*16+5*2];
              hand_data.finger[2].sensor[5]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[2].sensor[5]=sensor_Max_value;
              hand_data.finger[2].contact[5]=(bool)((uint8_t)buf[68+2*8+5]);  
              hand_data.finger[2].threadhold[5]=(uint8_t)sensor_thread_hold[2][5];     


              data_raw.u[1]=buf[4+2*16+6*2];
              data_raw.u[0]=buf[5+2*16+6*2];
              hand_data.finger[2].sensor[6]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[2].sensor[6]=sensor_Max_value;
              hand_data.finger[2].contact[6]=(bool)((uint8_t)buf[68+2*8+6]); 
              hand_data.finger[2].threadhold[6]=(uint8_t)sensor_thread_hold[2][6];      

              data_raw.u[1]=buf[4+2*16+7*2];
              data_raw.u[0]=buf[5+2*16+7*2];
              hand_data.finger[2].sensor[7]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[2].sensor[7]=sensor_Max_value;
              hand_data.finger[2].contact[7]=(bool)((uint8_t)buf[68+2*8+7]);  
              hand_data.finger[2].threadhold[7]=(uint8_t)sensor_thread_hold[2][7];   

              data_raw.u[1]=buf[4+3*16+0*2];
              data_raw.u[0]=buf[5+3*16+0*2];
              hand_data.finger[3].sensor[0]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[3].sensor[0]=sensor_Max_value;
              hand_data.finger[3].contact[0]=(bool)((uint8_t)buf[68+3*8+0]); 
              hand_data.finger[3].threadhold[0]=(uint8_t)sensor_thread_hold[3][0];       


              data_raw.u[1]=buf[4+3*16+1*2];
              data_raw.u[0]=buf[5+3*16+1*2];
              hand_data.finger[3].sensor[1]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[3].sensor[1]=sensor_Max_value;
              hand_data.finger[3].contact[1]=(bool)((uint8_t)buf[68+3*8+1]);  
              hand_data.finger[3].threadhold[1]=(uint8_t)sensor_thread_hold[3][1];       



              data_raw.u[1]=buf[4+3*16+2*2];
              data_raw.u[0]=buf[5+3*16+2*2];
              hand_data.finger[3].sensor[2]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[3].sensor[2]=sensor_Max_value;
              hand_data.finger[3].contact[2]=(bool)((uint8_t)buf[68+3*8+2]); 
              hand_data.finger[3].threadhold[2]=(uint8_t)sensor_thread_hold[3][2];        


              data_raw.u[1]=buf[4+3*16+3*2];
              data_raw.u[0]=buf[5+3*16+3*2];
              hand_data.finger[3].sensor[3]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[3].sensor[3]=sensor_Max_value;
              hand_data.finger[3].contact[3]=(bool)((uint8_t)buf[68+3*8+3]); 
              hand_data.finger[3].threadhold[3]=(uint8_t)sensor_thread_hold[3][3];        


              data_raw.u[1]=buf[4+3*16+4*2];
              data_raw.u[0]=buf[5+3*16+4*2];
              hand_data.finger[3].sensor[4]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[3].sensor[4]=sensor_Max_value;
              hand_data.finger[3].contact[4]=(bool)((uint8_t)buf[68+3*8+4]);  
              hand_data.finger[3].threadhold[4]=(uint8_t)sensor_thread_hold[3][4];       

              data_raw.u[1]=buf[4+3*16+5*2];
              data_raw.u[0]=buf[5+3*16+5*2];
              hand_data.finger[3].sensor[5]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[3].sensor[5]=sensor_Max_value;
              hand_data.finger[3].contact[5]=(bool)((uint8_t)buf[68+3*8+5]);  
              hand_data.finger[3].threadhold[5]=(uint8_t)sensor_thread_hold[3][5];       


              data_raw.u[1]=buf[4+3*16+6*2];
              data_raw.u[0]=buf[5+3*16+6*2];
              hand_data.finger[3].sensor[6]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[3].sensor[6]=sensor_Max_value;
              hand_data.finger[3].contact[6]=(bool)((uint8_t)buf[68+3*8+6]); 
              hand_data.finger[3].threadhold[6]=(uint8_t)sensor_thread_hold[3][6];        

              data_raw.u[1]=buf[4+3*16+7*2];
              data_raw.u[0]=buf[5+3*16+7*2];
              hand_data.finger[3].sensor[7]=data_raw.f;
              if(data_raw.f>sensor_Max_value)hand_data.finger[3].sensor[7]=sensor_Max_value;
              hand_data.finger[3].contact[7]=(bool)((uint8_t)buf[68+3*8+7]); 
              hand_data.finger[3].threadhold[7]=(uint8_t)sensor_thread_hold[3][7];        

              
              motor_position.u[3] = buf[103];
              motor_position.u[2] = buf[102];
              motor_position.u[1] = buf[101];
              motor_position.u[0] = buf[100];
              y_ = 37.9364 - ((double)motor_position.f / 16384.0000) * 2.0000;

              m_ = 0.0110808 * y_ * y_ + 0.000010032 * y_ + 0.79861393;

              b_ = 0.00377274 * y_ * y_ * y_ - 0.1195532 * y_ *y_ - 0.02988 * y_ + 0.9469;

              n_ = -0.0001983225 * y_ * y_ * y_ *y_ * y_ * y_ - 0.0009036 * y_ * y_ * y_ *y_ * y_ + 0.27925 * y_ * y_ * y_ *y_ + 1.3175425 * y_ * y_ *y_ +22.367225 * y_ * y_ + 93.856 * y_ + 106.1668;

              if(n_ >= 0)n_ = std::sqrt(n_);

              hand_data.motor_data.position = -1 * ((b_ - n_ ) / m_ + 7.235) * 2;

              motor_speed.u[1] = buf[105];
              motor_speed.u[0] = buf[104];   
                  
              hand_data.motor_data.speed=motor_speed.f * 0.1;
              
              if(hand_data.motor_data.speed < 1)hand_data.motor_data.speed = 0.00;

              hand_data.motor_data.voltage=((uint8_t)buf[106])*0.2;
              hand_data.motor_data.current=((uint8_t)buf[107])*0.03;
              hand_data.motor_data.tempreture=((uint8_t)buf[108])*0.4;
              hand_data.motor_data.motor_error=buf[109];  

              hand_data.motor_data.turn = (float)(motor_position.f); 

        
              while(READ_DATA == false)READ_DATA = true;

           }       

         for(int i = 0 ; i < 112 ; i++ )buf[i] = 0;
      }
}


void roshand_hardware::listen_data(uint8_t data_number, int max_seconds)
{

       boost::asio::deadline_timer timer( iosev); 

       char buf5[5];
       char buf8[8];
       char buf112[112];
       char buf37[37];

       if(data_number==5)
       {
            iosev.reset();
            boost::asio::async_read(*sp, boost::asio::buffer(buf5,sizeof(buf5)),boost::bind(&roshand_hardware::handle_read,this,buf5,_1, _2)) ;

        }          
      else if(data_number==8)
      {
            iosev.reset();
            boost::asio::async_read(*sp, boost::asio::buffer(buf8,sizeof(buf8)),boost::bind(&roshand_hardware::handle_read,this,buf8,_1, _2)) ;

        }

      else if(data_number==37)
      {
            iosev.reset();
            boost::asio::async_read(*sp, boost::asio::buffer(buf37,sizeof(buf37)),boost::bind(&roshand_hardware::handle_read,this,buf37,_1, _2)) ;


        }
      else if(data_number==112)
      {
            iosev.reset();
            boost::asio::async_read(*sp, boost::asio::buffer(buf112,sizeof(buf112)),boost::bind(&roshand_hardware::handle_read,this,buf112,_1, _2)) ;


        } 

       timer.expires_from_now(boost::posix_time::millisec(max_seconds)) ;      
       timer.async_wait(boost::bind(&boost::asio::serial_port::cancel,boost::ref(*sp)));


       try{

                 iosev.run();

       }
       catch(boost::system::system_error& ecc) {
                 std::cerr << ecc.what() << std::endl;

       }



}

//set sensor bias
void roshand_hardware::set_sensor_bias(int sensor_hi)
{
   uint8_t data_read[6] = {0xFF, 0x01, 0x05, 0x01, 0x00, 0x6D};  
   data_read[4] = ((uint16_t)sensor_hi) & 0xFF;

   boost::asio::write(*sp, boost::asio::buffer(&data_read[0], 6), ec);

   listen_data(5, 10);

}


void roshand_hardware::close_with_sensor(uint8_t close_step_mag, uint8_t open_step_mag)
{

   uint8_t data_read[7] = {0xFF, 0x01, 0x08, 0x02, 0x00, 0x00, 0x6D};  

   data_read[4] = close_step_mag;
   data_read[5] = open_step_mag;

   boost::asio::write(*sp, boost::asio::buffer(&data_read[0], 7), ec);

   listen_data(5, 20);

}
void roshand_hardware::open_with_sensor()
{


   uint8_t data_read[5] = {0xFF, 0x01, 0x09, 0x00,0x6D};  

   boost::asio::write(*sp, boost::asio::buffer(&data_read[0], 5), ec);
   listen_data(5, 20);

}

void roshand_hardware::set_position_without_sensor(uint32_t msg)
{

   uint32_t target_position;   

   uint8_t data_read[8] = {0xFF, 0x01, 0x07, 0x03, 0x00, 0x00, 0x00, 0x6D};  
   target_position=msg;
   data_read[4] = (target_position >> 16 ) & 0xFF;
   data_read[5] = (target_position >> 8 ) & 0xFF;
   data_read[6] = (target_position ) & 0xFF;

   boost::asio::write(*sp, boost::asio::buffer(&data_read[0], 8),ec);

   listen_data(5, 20);

}
void roshand_hardware::read_data()
{


   uint8_t data_read[5] = {0xFF, 0x01, 0x03, 0x00,0x6D};  

   boost::asio::write(*sp, boost::asio::buffer(&data_read[0],5),ec);
   listen_data(112,5);

}

void roshand_hardware::calibrate_data()
{

   uint8_t data_read[5] = {0xFF, 0x01, 0x0A, 0x00,0x6D};  

   boost::asio::write(*sp, boost::asio::buffer(&data_read[0],5),ec);

   listen_data(5, 5);

}



bool roshand_hardware::spOnce(uint32_t target_position, bool use_sensor, bool use_without_action, bool control_finished , uint8_t close_step, uint8_t open_step)
{

    if(use_sensor == true && control_finished == false && use_without_action == true)close_with_sensor(close_step, open_step);
    else if(use_sensor == false && control_finished == false && use_without_action == true)set_position_without_sensor(target_position);
    return true;

};



}
