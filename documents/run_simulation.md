# Run simulation
In this section, we will show you how to run the simulation in the docker container. We will use the CoppeliaSim platform to simulate the Fetch robot's movement and control it using ROS. 

First of all, you need to have two terminals. One is for the simulation, while the other is for the ROS controllers which will receive the commands and send them to the robot in the simulation. Therefore, in two terminals, you need to run the following commands.

```
roslaunch fetch_coppeliasim simulation.launch
```
```
roslaunch fetch_coppeliasim fetch_control.launch
```

After you launch the simulation, you should see a table with a set of objects on it. The Fetch robot will be placed in front of the table. You can control the robot using the ROS controllers. You can also run the following command to control the robot's base movement.

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _key_timeout:=0.05
```