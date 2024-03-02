# CoppliaSim Platform for Fetch Robot

We are using the Fetch robot in the CoppeliaSim platform to simulate the robot's movement and control it using ROS.

## Description

The Fetch robot is a mobile manipulator designed to automate material handling and transportation in logistics and manufacturing facilities. It is a mobile base with a robotic arm and a gripper. The robot is equipped with a variety of sensors, including a 3D camera, a depth sensor, and a laser scanner. The robot is controlled using ROS, which provides a set of tools and libraries for building robot applications. In this project, we can deploy our scene modeling algorithm and test the manipulation planning system in the CoppeliaSim platform. Besides, we provide a dockerfile to build the environment for the CoppeliaSim and ROS, so we support the user to run the simulation in the docker container.

## Usage

1. [<h2> Prepare the environment </h2>](documents/prepare_environment.md)

    Build the docker image and run the docker container.

2. [<h2> Run the simulation </h2>](documents/run_simulation.md)

    Run the simulation in the docker container.

3. [<h2> Add Objects to the Scene </h2>](documents/add_objects_to_scene.md)

    Add objects to the scene for modeling and manipulation planning.

## Authors
Jiaming Hu