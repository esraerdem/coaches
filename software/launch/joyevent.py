#!/usr/bin/env python
# encoding: utf-8

# ROS imports
import roslib; #roslib.load_manifest('coaches')
import rospy

# ROS msg and srv imports
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


# Python Libraries
import sys
import traceback

###  Variables  ###
LINEAR_SPEED = 0.8
ANGULAR_SPEED = 0.6

###  Classes  ###

class Joy2Twist(object):
    joyEnabled = True

    """Joy2Twist ROS Node"""
    def __init__(self):
        # Initialize the Node
        rospy.init_node("Joy2Twist")
        
        # Setup the Joy topic subscription
        self.joy_subscriber = rospy.Subscriber("/diago/joy", Joy, self.handleJoyMessage, queue_size=1)
               
        self.PNP_event_pub =rospy.Publisher('/diago/PNPConditionEvent', String, queue_size=1)
        # Spin
        rospy.spin()
    
    def handleJoyMessage(self, data):
        """Handles incoming Joy messages"""
        
        
        if len(data.buttons) > 0:
         
          if (data.buttons[0]==1): # blue = personPrinter
              self.PNP_event_pub.publish("personPrinter")
          if (data.buttons[1]==1): # green = personHere
              self.PNP_event_pub.publish("personhere")
          if (data.buttons[2]==1): # red = helpbringdoc
              self.PNP_event_pub.publish("helpbringdoc")
          if (data.buttons[3]==1): # orange = helptechnician
              self.PNP_event_pub.publish("helptechnician")
    

###  If Main  ###
if __name__ == '__main__':
    try:
        Joy2Twist()
    except:
        rospy.logerr("Unhandled Exception in the joy2Twist Node:+\n"+traceback.format_exc())
