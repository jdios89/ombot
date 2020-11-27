# README #

Here is the Ombot [Omnidirectional mobile robot] source code. It is also included libraries and code for the Arduino DUE, that acts as the base controller. 

![Screenshot](ombot.png)

Later it will be included the electronics schematics and 3d printed supports. 

### What is this repository for? ###

* Quick summary
* Version 1.1
* [Learn Markdown](https://bitbucket.org/tutorials/markdowndemo)

### How do I get set up? ###

* In the launch folder there are several launch files:
  
```
#!c++

roslaunch my_repository base_controller.launch
```
Will initiate the Arduino communication. Will setup the nodes for the Odometer, Speed commands, IMU and UL Range sensor. 

```
#!c++

roslaunch my_repository camera_laser.launch
```
Will initiate the Asus Xtion Pro Live camera node as well as the fake laser transformed from the depth image. Notice: Default camera calibration is initiated. 

```
#!c++

roslaunch my_repository om_bot.launch
```
WIll initiate the above launch files and the robot joint and state publisher. It also provides mesh visualization. 

```
#!c++

roslaunch teleop_twist_joy teleop.launch
```
Will initiate the ps3 joystick communication and node to send twist commands to the base controller. 

```
#!c++

roslaunch my_repository amcl_omni.launch
```
Will initiate the amcl localisation package with its parameters and the map. 

```
#!c++

roslaunch my_repository move_base2.launch
```

WIll initiate the move_base package and the amcl_omni.launch to navigate in the supplied map.

```
#!c++

roslaunch my_repository display2.launch
```
WIll initiate RVIZ to visualize the robot in the map and to send visual navigation goals. 

* Dependencies
* Database configuration
* How to run tests
* Deployment instructions

