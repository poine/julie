# julie_sim

This repository contains everything to the simulation of Julie the golf cart


## julie_gazebo

## julie_gazebo_bringup

Contains several launch files in order to launch some or all the components of the gazebo simulations.

example:
```
roslaunch julie_gazebo_bringup julie_maze.launch robot_model:=julie2 world_name:=maze_1.world
```

## julie_gazebo_ackermann_controller

Julie’s Gazebo Ackermann plug-in controller.
Implements the control of the ackerman kinematics of the robot, controlling the traction and steering motors.
Publishes the robot’s odometry.

## julie_sim_nav
Contains launch file to start different configuration of the navigation stack in simulation

```
roslaunch julie_sim_nav sbpl_and_teb_in_gazebo.launch world_name:=maze_1.world
```

## julie_stage

