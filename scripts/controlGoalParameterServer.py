#!/usr/bin/env python


import sys
import math
import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from pyorca import Agent
from pyorca import orca

class Turtlebot():
    def __init__(self):
        self.goals = []
        self.currentGoalIndex = 1
        self.markerSub = rospy.Subscriber('/marker', Marker, self.callbackMarkers)
        self.collidingAgents = []
        self.robotAgent = Agent((0, 0), (0, 0), 0.25, 1, (0,0))
        
    # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.listener = tf.TransformListener()
     
        rospy.Subscriber("/publish_path", Path, self.receivedPath)
        
    def callbackMarkers(self, marker):
        self.collidingAgents = []
        for p in marker.points:
            agent = Agent((p.x, p.y), (0, 0), 0.01, 0, (0,0))
            self.collidingAgents.append(agent)


    
    def receivedPath(self, paths):
        for path in paths.poses:
            dic = {}
            dic['x'] = path.pose.position.x
            dic['y'] = path.pose.position.y
            tup = ("goal", dic)
            self.goals.append(tup)
            


    def command(self, rate):
        goal = PointStamped()
        base_goal = PointStamped()
        goal.header.frame_id = "odom"
        goal.header.stamp = rospy.Time()
        if len(self.goals) < 1 or self.currentGoalIndex >= len(self.goals):
            self.publish(0,0)
            rospy.loginfo("Objetivos alcanzados")
        else:
            goal.point.y = self.goals[self.currentGoalIndex][1]['y']
            goal.point.x = self.goals[self.currentGoalIndex][1]['x']
            goal.point.z = 0.0
            rospy.loginfo("-------------OBJ: " + str(goal.point.x) + " Y:" + str(goal.point.y))
            try:
                base_goal = self.listener.transformPoint('base_footprint', goal)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("Problem TF")
                return
            
            angular = math.atan2(base_goal.point.y, base_goal.point.x)
            degrees = math.degrees(angular)
            if degrees > 5:
                angular = 0.5
                linear = 0.2
            elif degrees < -5:
                angular = -0.5
                linear = 0.2
            else:
                angular = degrees * 0.01 * 0.35
                linear = 0.4
            
            if math.sqrt((base_goal.point.x)**2+(base_goal.point.y)**2) < 0.5:	#Check if we reached the goal
                self.currentGoalIndex = self.currentGoalIndex + 1
            
            x_vel = math.cos(angular) * linear
            y_vel = math.sin(angular) * linear
            self.robotAgent = Agent((0, 0), (self.robotAgent.velocity[0], self.robotAgent.velocity[1]), 0.25, 0.001, (x_vel,y_vel))
            [newVel, lines] = orca(self.robotAgent, self.collidingAgents, 1, 0.1)
            x_vel = newVel[0]
            y_vel = newVel[1]
            self.robotAgent = Agent((0, 0), (x_vel, y_vel), 0.25, 0.001, (x_vel,y_vel))
            
            #if (math.atan2(y_vel, x_vel) - angular > 0.2 or math.atan2(y_vel, x_vel) - angular < -0.2):
                #linear = -0.2
            #else:
            linear = math.sqrt(x_vel**2.0 + y_vel**2.0)
                
            angular = angular + (math.atan2(y_vel, x_vel) - angular) * 10
            angular = min(angular, 0.8)
            angular = max(angular, -0.8)
            #if linear < 0:
                #angular = -angular
            
            self.publish(linear,angular)

    def publish(self,lin_vel, ang_vel):
    # Twist is a datatype for velocity
        move_cmd = Twist()
    # let's go forward at 0.2 m/s
        move_cmd.linear.x = lin_vel
    # let's turn at 0 radians/s
        move_cmd.angular.z = ang_vel

        self.cmd_vel.publish(move_cmd)
        
    def shutdown(self):
        #goalse stop command prior to shutting down the script
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:

        rospy.init_node('robotcontrol', anonymous=False)

        #goals = rospy.get_param("/control/path")
        #goals=sorted(goals.items())
        #rospy.loginfo(str(goals))

        rospy.loginfo("To stop TurtleBot CTRL + C")
        
        robot=Turtlebot()
  
        rospy.on_shutdown(robot.shutdown)

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            robot.command(r)
            r.sleep()

    except Exception, e:
        rospy.loginfo("robotcontrol node terminated. " + str(e))
