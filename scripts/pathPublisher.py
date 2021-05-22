#!/usr/bin/env python
# license removed for brevity
import rospy
import yaml
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from dijkstra import Dijkstra

class PathPlanner:
    
    def __init__(self):
        self.pathPublisher = rospy.Publisher('/publish_path', Path, queue_size=10)
        self.costmap_topic = rospy.get_param(
            '~costmap_topic', default="costmap_2d/costmap/costmap")
        self.mapSubscriber = rospy.Subscriber(self.costmap_topic, OccupancyGrid, self.path_callback)
        self.firstCall = False
        
        self.goal_path = rospy.get_param('~goals', default="goals.yml")
    
        self.initx = float(rospy.get_param('~initial_pose_x'))
        self.inity = float(rospy.get_param('~initial_pose_y'))

    
        self.goalx = float(rospy.get_param('~goal/x'))
        self.goaly = float(rospy.get_param('~goal/y'))
        
        if rospy.has_param('~world'):
            self.world = rospy.get_param("~world")
        else:
            self.world = 0
        
        
    def path_callback(self, map):
        
        
            
        self.map = map
        if self.firstCall == False:
            rospy.loginfo("+++++++++++++++++initX: " + str(self.initx) + " initY: " + str(self.inity) + " goalX: " + str(self.goalx) + " goalY: " + str(self.goaly))
            self.firstCall = True
            
            if self.world != 0:
            
                dijkstra = Dijkstra(self.map)
                self.path = dijkstra.planning(self.initx, self.inity, self.goalx, self.goaly)
                self.save_as_yaml(self.path)
                x, y = self.path
                x = x[::-1]
                y = y[::-1]
                my_path = Path()
                my_path.header.frame_id = "map"
                my_path.header.stamp = rospy.get_rostime()
                size = range(len(self.path[0]))
                poses = []
                for i in size:
                    p_stamp = PoseStamped()
                    p_stamp.pose.position.x = x[i]
                    p_stamp.pose.position.y = y[i]
                    p_stamp.pose.position.z = 0
                    p_stamp.header.frame_id = "map"
                    p_stamp.header.stamp = rospy.get_rostime()
                    poses.append(p_stamp)
                my_path.poses = poses
                self.pathPublisher.publish(my_path)
            else:
                poses = []
                self.path = [(self.goalx, self.goaly)]
                my_path = Path()
                my_path.header.frame_id = "map"
                my_path.header.stamp = rospy.get_rostime()
                for i in range(2):
                    p_stamp = PoseStamped()
                    p_stamp.pose.position.x = self.goalx
                    p_stamp.pose.position.y = self.goaly
                    p_stamp.pose.position.z = 0
                    p_stamp.header.frame_id = "map"
                    p_stamp.header.stamp = rospy.get_rostime()
                    poses.append(p_stamp)
                my_path.poses = poses
                self.pathPublisher.publish(my_path)
            
            
            
            
            
    def save_as_yaml(self, path):
        goalsList = {}
        x, y = path
        x = x[::-1]
        y = y[::-1]
        x_str = "x"
        y_str = "y"
        size = range(len(path[0]))
        for i in size:
            aux = "goal"+str(i)
            aux_dic = {}
            aux_dic[x_str] = x[i]
            aux_dic[y_str] = y[i]
            goalsList[aux] = aux_dic
        goalsYaml = {}
        goalsYaml["path"] = goalsList
        with open(self.goal_path, 'w') as file:
            documents = yaml.dump(goalsYaml, file)
        print "Guardado archivo en " + self.goal_path
        
if __name__ == '__main__':
    try:
        
        rospy.init_node('PathPlanner', anonymous=True)
        
        pathPlanner = PathPlanner()
        
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

    except rospy.ROSInterruptException:
        pass
    

