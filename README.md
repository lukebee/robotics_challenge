# Robotics Challenge #1
Robotic challenge scenarios for the Robotics and Advanced subject of UPO's Software Engineering Grade.

## Authors

```
Alvaro Navarro Mora & Luka Bubalo
```

The repository can be found in https://github.com/lukebee/robotics_challenge

## Installation and Configuration

- Clone the repository in your ~/catkin_ws/ folder.
- Run rosrun robotics_challenge install.bash
- In a terminal, run source ~/catkin_ws/devel/setup.bash and move inside the catkin_ws folder.
- Run catkin_make.
- In a new terminal run source /opt/ros/melodic/setup.bash  and roscore.
- You are ready to run the maps now. You can see the different commands for running the maps in section How to run.

The initial coordinates and the final goals can be specified in the parameter contained in the file /launch/robotics_challenge.launch. The parameters are init_x, init_y init_a, goal_x and goal_y. Take in consideration that initial points will be overwritten in map number 3.

## How to run?
There are three different world that can be run in terms of this challenge:

```
$  rosrun robotics_challenge robotics_challenge_1.bash
```

```
$  rosrun robotics_challenge robotics_challenge_2.bash
```

```
$  rosrun robotics_challenge robotics_challenge_3.bash
```

## Difficulties
So far we have encountered not that small number of either small or major difficulties and majority of them we were able to solve, and the rest, unfortunately, we weren’t. 
Here we will make a list of the major issues we stumbled upon since the small issues (eg. file permission issues, package installation in the first version, etc.) were easy to solve. At first we got stuck understanding how ORCA works and how some parameters should be adjusted as well as their meaning and impact on the algorithm's output. Also, regarding the obstacle avoidance we had challenges in adjusting the angular and linear velocity depending on ORCA’s output so the robot doesn’t do any weird and unexpected things. We have had many trials and errors in order to adjust velocities, some of them have been working better, but in the end we haven’t managed to adjust it. Furthermore, we were facing issues with A* algorithm and its parameter (value in costmap.data) which caused the algorithm to be stuck in an infinite loop. After many attempts, we have decided to disable A* algorithm on the first two maps.  

## Path planning
For path planning we are using pathPublisher.py node which takes initial and goal coordinates as well as costmap and sends those values to dijkstra.py. After A* finds the path we transform this path in series of points and publish those to controlGoalParameterServer.py. Also, we have been publishing those points to the yaml file for testing purposes. The important note about the first two worlds is that they
don't use A* algorithm for finding the optimal path since we haven't managed to adjust the A* parameter (value in costmap.data used ) in a way that it works with every mentioned world.

## Obstacles detection and avoidance
- orca: we are  using this algorithm to avoid the obstacles detected by the laser.
- laser downsample: this is the class used for subscribing to the laser and publishing the obstacles, so the robot can avoid them using the mentioned orca algorithm.
- There are some cases in which the orca algorithm is not enough to avoid the obstacles, and it is needed an additional algorithm to sort them out. We have decided to use the A* algorithm for this, but as commented before, we have only managed to get it working for map number 3, as in maps number 1 and 2 it does not converge to a valid path.
