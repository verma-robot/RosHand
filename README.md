
## file system

 - RosHand_msg`: all the messages, servers and actionlib format are defined here.
 - roshand_demo : two demo programs with C++
 - roshand_driver : driver of roshand gripper
 - roshand_visualizer : robot urdf models and meshes are stored here. 

## Installation

To make roshand-ros part of your workspace, follow these steps :

  ```
  cd ~/catkin_ws/src
  git clone https://github.com/verma-robot/RosHand.git roshand-ros
  cd ~/catkin_ws
  catkin_make

  ```
To access the arm via the command:

  ```
  sudo chmod 666 /dev/ttyUSB0
  
  ```

### Launch driver

`bringup.launch` in `roshand_driver` folder launches the gripper. you can launch the Roshand gripper by using the following step:

  ```
  roslaunch roshand_driver bringup.launch 

  ```

### control roshand gripper with actionlib 

  ```
  rosrun roshand_demo ActionDemo

  ```

### control roshand gripper with service 

  ```
  rosrun roshand_demo ServiceDemo

  ```

### calibrate the touch sensor 

  ```
  rosservice call  /calibrate_sensor_service 

  ```

### rviz

  `dispaly.launch` in `roshand_visualizer` folder can see the gripper in RVIZ, you can use the following step:

  ```

   roslaunch roshand_visualizer dispaly.launch 

  ```
 - paramters : 

   **with_real_robot** --if you have a real roshand gripper , set it to true; if not, set it to false;

### gazebo

  `gazebo.launch` in `roshand_visualizer` folder can launch the gripper in gazebo

 ```

   roslaunch roshand_visualizer gazebo.launch 

  ```


## Report a Bug
  Any bugs, issues or suggestions may be sent to support@verma-robot.com

  Thanks!
