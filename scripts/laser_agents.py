#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from pyorca import Agent
import numpy as np

class LaserDownSample:
    def __init__(self):
        self.pub = rospy.Publisher('~agent', Agent, queue_size=10)
        self.sub = rospy.Subscriber('scan', LaserScan, self.callback)
    def callback(self, laser):
        angle_vector = np.arange(laser.angle_min, laser.angle_max, laser.angle_increment)
        for i in range(len(angle_vector)):
            d = laser.ranges[i]
            if d < 10:
                theta = angle_vector[i]
                x = d * np.cos(theta)
                y = d * np.sin(theta)
                agent = Agent((x, y), (0, 0), 1, 10, 0, 0)
                console.log("Test")
                self.pub.publish(agent)

if __name__ == '__main__':
    try:
        rospy.init_node('LaserDownSample', anonymous=True)
        laser = LaserDownSample()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass