# Multi-radar inertial odometry

This is the implementaion of the paper "Multi-Radar Inertial Odometry for 3D State Estimation using mmWave Imaging Radar" tested with two 4d automotive radars.

[demo video](https://youtu.be/Nd2S2k3zKCg)

## Dependency

- ROS (tested with melodic)
- gtsam
- Ceres solver
- PCL

## Install

```bash
cd ~/catkin_ws/src
git clone git@github.com:jerry8137/multi-radar-inertial-odometry.git
cd ..
catkin_make
```

## Run

```bash
source devel/setup.bash
roslaunch radar_odometry radar_odometry_offline_test.launch
```


## Acknowledgement

- The implementation of radar_odometry is modified from LIO-SAM.
