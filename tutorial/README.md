# Fenswood Volcano Template Tutorials

## Introduction

This repository provides a template and tutorial to get you started writing code and running simulations for the Group Project in Aerial Robotics, based on the Fenswood Volcano scenario.

In this tutorial, you will learn how to write code in Python to interface with the (simulated) drone using ROS.  In particular, you will learn the newer ROS2 methods, which offer improved security and scalability.  On the way, you will learn the fundamentals of controlling a drone through a standard autopilot.

> You will also need to do little bits of Linux and Docker 'magic' to make the application work.  You are not expected to master these aspects: ask for help if they cause problems.  The primary learning objectives here are drone control, Python and ROS.  If you are interested in more details, see the [Starling overview](starling.md).

In the examples, a drone takes off, moves to a target location, and then flies home.  In the later `perception` example, some very basic information is gathered from its camera image.  Together, these examples give you all the building blocks you will need to carry out the project. 

## How to use this tutorial

Inspired by the [minimal ROS2 examples](https://github.com/ros2/examples/tree/master/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber), this tutorial takes you through several iterations of code with almost the same functionality.  A full working example is provided in each case with a detailed description of the code.  The section below shows the contents and learning obectives for each stage of the tutorial.  If you are really confident on ROS concepts and object-oriented programming in Python, go straight to the ROS timer tutorial.  Otherwise, you are advised to simply work through from start to finish _but_ you might find some stages easier than others, depending on your past experience.  

Each tutorial stage comes with some exercises for you to add to the code yourself.  To test your solutions, the simulation is always started using `docker-compose [-f <some-file>] up --build` and then stopped using `Ctrl+C`.  Use the terminal window output, foxglove studio, or the Docker Desktop (Windows and Mac only) to inspect the results.  Don't forget the `--build`.  Otherwise, your changes don't get built into the simulation, and you're just running the same code over and over.

There are countless links in the tutorials.  These provide extra information, background and sources, but are not essential reading.  Follow them if you are interested, but do not feel compelled to digest each one.

## Contents

1. [Getting started](getting_started.md)
    - Learn how to install and run the simulation

1. [Drone control](drone_control.md)
    - Learn the key steps in flying a drone using Ardupilot and ROS.

2. [Old school](old_school.md)
    - Learn how to control the drone using ROS from Python

3. [A simple class](simple_class.md)
    - Learn how to use a Python class as a better drone controller

4. [A better, more modular class](modular.md)
    - Learn how to use a more modular programming approach to improve your Python class

5. [Finite state machine](finite_state.md)
    - Learn how to use a finite state machine as your decision-making engine

6. [ROS timer](ros_timer.md)
    - Learn how to use ROS2 built-in timer to manage the finite state machine execution

7. [Perception](perception.md)
    - Learn how to add a second process or 'node' to the ROS application (ROS)
    - Learn how to install a third-party library in the Volcano application (Docker)
    - Learn how to access the camera image ready for vision processing (OpenCV)

 




