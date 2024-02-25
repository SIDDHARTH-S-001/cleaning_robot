#! /usr/bin/python3

import rospy
from nav_msgs.msg import Path

def callback(data):
    # This function will be called whenever a message is received on the topic
    # '/move_base/SpiralSTC/plan'
    # You can process the received data here
    # For example, print the received path
    rospy.loginfo("Received path: %s", data)

def listener():
    # Initialize the ROS node
    rospy.init_node('path_subscriber', anonymous=True)

    # Subscribe to the topic '/move_base/SpiralSTC/plan' with the message type 'Path'
    rospy.Subscriber("/move_base/SpiralSTC/plan", Path, callback)

    # Spin() keeps your code from exiting until the node is shutdown
    rospy.spin()

if __name__ == '__main__':
    listener()