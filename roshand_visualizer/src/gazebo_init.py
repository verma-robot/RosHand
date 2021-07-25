#! /usr/bin/env python
"""Publishes joint trajectory to move robot to given pose"""

import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Empty
import argparse
import time

def moveJoint (jointcmds,prefix,nbJoints):
  topic_name = '/left_arm_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
  jointCmd = JointTrajectory()  
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(1.0);  
  point.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0, nbJoints):
    jointCmd.joint_names.append('left_joint_'+str(i+1))
    point.positions.append(jointcmds[i])
    point.velocities.append(0)
    point.accelerations.append(0)
    point.effort.append(0) 
  jointCmd.points.append(point)
  rate = rospy.Rate(100)
  count = 0
  while (count < 50):
    pub.publish(jointCmd)
    count = count + 1
    rate.sleep()     


if __name__ == '__main__':
  try:    
    rospy.init_node('roshand_init')	
    prefix='left'
    nbJoints=6
    nbfingers=2	

    time.sleep(1)

    # Unpause the physics
    #rospy.wait_for_service('/gazebo/unpause_physics')
    #unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    #resp = unpause_gazebo()

    #if (nbJoints==6):
      #home robots
    moveJoint ([0.0,1.2,2.0,-2.2,1.4,0.0],prefix,nbJoints)
    #else:
      #moveJoint ([0.0,2.9,0.0,1.3,4.2,1.4,0.0],prefix,nbJoints)
    #time.sleep(5)
    moveFingers ([0.2,0.2],prefix,nbfingers)

    nbfingers=1
    moveFinger1Tips([0.2],prefix,nbfingers)
    moveFinger2Tips([0.2],prefix,nbfingers)
  except rospy.ROSInterruptException:
    print "program interrupted before completion"
