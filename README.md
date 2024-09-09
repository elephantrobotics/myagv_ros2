# myagv_ros2
ROS2 packages for myAGV

# Checklist

- [] myagv_description
- [] myagv_navigation2
- [] myagv_teleop
- [] myagv_odometry
- [] myagv_cartographer
- [] navigation2
- [] ydlidar_ros2_driver
- [] Gazebo simulation

# Installation

You need to have previously installed ROS2.

Create workspace and clone the repository.

```bash
mkdir ~/myagv_ros2/src
cd ~/myagv_ros2/src
git clone -b jazzy-Pi5 https://github.com/elephantrobotics/myagv_ros2.git
```

Install dependencies and build workspace
```
cd ~/myagv_ros2
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Setup the workspace
```
source ~/myagv_ros2/install/setup.bash
```
