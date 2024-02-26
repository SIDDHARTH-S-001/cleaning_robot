#!/usr/bin/python3

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import ast

class PathSubscriber:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('path_subscriber', anonymous=True)
        rospy.Subscriber("coverage_points", String, self.callback)
        self.path_publisher = rospy.Publisher("coverage_path", Path,  queue_size=10)
        self.path_points = []
        self.rate = rospy.Rate(20)

    def callback(self, data):
        rospy.loginfo(f"Received Points")
        self.path_points = ast.literal_eval(data.data)

    def spin(self):
        while True:
            self.publish_path()
            self.rate.sleep()

    def send_goal(self):
        pass

    def follow_path(self):
        if self.path_points is not None:
            for path in self.path_points:
                for point in path:
                    pass

    def publish_path(self):
        if self.path_points is not None:
            for path in self.path_points:
                msg = Path()
                msg.header.frame_id = 'map'
                msg.header.stamp = rospy.Time.now()
                for point in path:
                    point_msg = PoseStamped()
                    point_msg.header.frame_id = 'map'
                    point_msg.header.stamp = rospy.Time.now()
                    point_msg.pose.position.x = point[0]
                    point_msg.pose.position.y = point[1]
                    point_msg.pose.orientation.w = 1
                    msg.poses.append(point_msg)
                self.path_publisher.publish(msg)
                self.rate.sleep()
                rospy.loginfo("Published Plan")

if __name__ == '__main__':
    path_subscriber = PathSubscriber()
    path_subscriber.spin()
