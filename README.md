# Jaguar Control
This package provides CLI and ROS interface to control [Jaguar V4 with Manipulator Arm Mobile Robotic Platform](http://jaguar.drrobot.com/specification_V4Arm.asp)

## Features
* velocity and joints control with a keyboard
* velocity and joints control through ROS
* visualization and publishing of the robot sensor and configuration state
* limits for flipper and arm rotations (+/- 45 degrees)
* memorization of the joint configuration
* reconfiguration to the initial state `flippers and arm extended` on startup
* low battery and high motor current warnings


## Requirements  

* [Go](https://golang.org/) >= 1.15  
* [Python](https://www.python.org/) >= 3.8.5  
* [ROS](https://www.ros.org/) Noetic/Melodic/Kinetic
* [RabbitMQ](https://www.rabbitmq.com/) >= 3.6.10  

## Install  

```bash  
go get github.com/AndreiMitriakov/jagcontrol  
```

## Usage

```bash  
go run main.go
```
## ROS
Launch `scripts/middleware.py` and be sure that `roscore` is running.
### Topics
All data is published in String message in json format.
##### Publish
* Configuration `/robot/state`
* Sensor data `/robot/sensor_state`
##### Subscribe
* Close connection `/robot/command` <- String(0.0)
* Arm change angles (rad) `/robot/cmd_arm` <- Float32MultiArray([arm1, arm2])
* Set velocity (-1 ... 1), flipper change angles (rad) `/robot/cmd_array` <- Float32MultiArray([linear, angular, front_flipper, rear_flipper])
