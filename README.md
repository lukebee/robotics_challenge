# Robotics Challenge #1
Robotic challenge scenarios for the Robotics and Advanced subject of UPO's Software Engineering Grade.

## Authors

```
Alvaro Navarro Mora & Luka Bubalo
```

## Installation instructions

```
rosrun robotics_challenge install.bash
```

## Configuration

## Path planning
The important note about the first two worlds is that they
don't use A* algorithm for finding the optimal path since we haven't managed to adjust the A* parameter (value in costmap.data used ) so it works
with every mentioned world.

## Obstacles detection and avoidance
- orca
- laser downsample
- not working always explanation

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
