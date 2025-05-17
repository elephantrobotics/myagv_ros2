# myagv_ros2
ROS2 packages for myAGV

> Software environment for Jetson Nano

```
ros2 galactic
Jetpack 4.6
Opencv：4.8.0 with CUDA：YES
```

# Checklist

- [x] myagv_description
- [x] myagv_navigation2
- [x] teleop_twist_keyboard
- [x] myagv_odometry
- [ ] myagv_cartographer
- [ ] slam_toolbox
- [x] slam_gmapping
- [x] navigation2
- [x] ydlidar_ros2_driver
- [ ] Gazebo simulation
- [ ] myagv_multi

# Installation

You need to have previously installed ROS2.

Create workspace and clone the repository.

```bash
git clone -b galactic-JN https://github.com/elephantrobotics/myagv_ros2.git myagv_ros2/src
```

Install dependencies
```
cd ~/myagv_ros2

rosdep install --from-paths src --ignore-src -r -y
```
```
sudo apt install ros-galactic-bondcpp \
    ros-galactic-test-msgs* \
    ros-galactic-behaviortree-cpp-v3* \
    ros-galactic-ompl \
    ros-galactic-joint-state-publisher \
    ros-galactic-rqt-tf-tree \
    ros-galactic-diagnostic-updater \
    ros-galactic-camera-info-manager -y
```

Build workspace

```
cd ~/myagv_ros2

colcon build
```

Setup the workspace
```
source ~/myagv_ros2/install/local_setup.bash
```

# Update to new version
```
cd ~/myagv_ros2/src

git pull

cd ..

colcon build
```

