# adaframe

## Table of contents
* [General info](#general-info)
* [Installation](#installtion)
* [How to use adaframe on a simulated vehicle](#how-to-use-adaframe-on-a-simulated-vehicle)

## 1 General info

Adaframe is a Python/ROS framework for running adaptive software on autonomous vehicles running on DUNE.

## 2 Installation

###  2.1 ROS

[Follow the steps here](http://wiki.ros.org/ROS/Installation).

###  2.2 DUNE

DUNE (Unified Navigation Environment) is a runtime environment for unmanned system's on-board software.

```
$ mkdir -p ~/dune_all/build
$ cd dune_all
$ git clone https://github.com/NTNU-Adaptive-Sampling-Group/dune.git
$ cd build
$ cmake ../dune
$ make -j8
```

For more information, see [github.com/LSTS/dune](https://github.com/LSTS/dune).

###  2.3 Neptus

```
$ git clone https://github.com/NTNU-Adaptive-Sampling-Group/neptus.git
$ cd neptus
$ ./neptus
```

For more information, see [github.com/LSTS/neptus](https://github.com/LSTS/neptus).

### 2.4 Create a catkin workspace

First, create and build a catkin workspace. If you already have one, skip this step.

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
```


### 2.5 Install adaframe

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/NTNU-Adaptive-Sampling-Group/adaframe_examples.git
$ git clone https://github.com/NTNU-Adaptive-Sampling-Group/imc_ros_interface.git
$ cd ..
$ catkin_make
```

## How to use adaframe with a simulated vehicle

When opening a new terminal, remember to source setup.bash!

```
$ cd ~/catkin_ws/
$ source devel/setup.bash
```

Start a simulation of an LAUV vehicle:

```
$ cd ~/dune_all/build
$ ./dune -c lauv-simulator-1 -p Simulation
```

Start up the bridge node:

```
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ cd src
$ roslaunch imc_ros_interface/launch/bridge.launch 
```

Run example code:
```
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosrun adaframe_examples auv_governor_example.py 
```
or
```
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosrun adaframe_examples gp_example.py 
```
Congratulations, you have an adaptive AUV!

## Starting from Neptus

In Neptus, go to View->Plugin Manager and add the "FollowReference Interaction for NTNU". A small NTNU-logo should appear in the bar in the upper left-hand corner of Neptus. Press this button, you should now be able to right click the map and press "Activate Follow Reference for <vehicle name>", if you have done the above steps correctly, the vehicle should now start to move. 


## Writing you own code
Depending on how fancy you want to be, you can create a new ROS package or just copy one of the examples and take it from there. Do NOT rewrite and commit/push the examples unless you are making them more clear or debugging them. If you want to make your own code from the examples, remember to rename it. When you are happy with your code, feel free to push it to Git and share with the others. 


