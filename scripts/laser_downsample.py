#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import numpy as np

class LaserDownSample:
    def __init__(self):
        self.pub = rospy.Publisher('/marker', Marker, queue_size=10)
        self.sub = rospy.Subscriber('scan', LaserScan, self.callback)
        
    def callback(self, laser):
        angle_vector = np.arange(laser.angle_min, laser.angle_max, laser.angle_increment)
        marker = Marker()
        for i in range(len(angle_vector)):
            d = laser.ranges[i]
            if d < 10:
                point = Point()
                theta = angle_vector[i]
                point.x = d * np.cos(theta)
                point.y = d * np.sin(theta)
                point.z = 0
                
                marker.header = laser.header
                marker.points.append(point)
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.1
        marker.color.a = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.pose.orientation.w = 1
        marker.type = marker.SPHERE_LIST
        marker.ns = 'laser'
        self.pub.publish(marker)

if __name__ == '__main__':
    try:
        rospy.init_node('LaserDownSample', anonymous=True)
        laser = LaserDownSample()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass