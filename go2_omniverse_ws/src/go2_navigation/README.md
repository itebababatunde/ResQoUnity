# Go2 Navigation Package for Isaac Sim

This package provides SLAM and autonomous navigation capabilities for Go2 robots in Isaac Sim simulation using ROS2 Nav2 and SLAM Toolbox.

## 🎯 Features

- **SLAM Mapping**: Real-time map building using SLAM Toolbox
- **Autonomous Navigation**: Full Nav2 stack integration
- **Multi-Robot Support**: Compatible with multiple robot namespaces (robot0, robot1, etc.)
- **Isaac Sim Integration**: Works seamlessly with your existing simulation

## 📋 Prerequisites

```bash
# Install required ROS2 packages
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-rviz2
```

## 🚀 Quick Start

### Step 1: Build the Package

```bash
cd ~/ResQoUnity/go2_omniverse_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_navigation
source install/setup.bash
```

### Step 2: Start Isaac Sim

In **Terminal 1**:
```bash
cd ~/ResQoUnity
./start_simulation.sh
```

Wait until Isaac Sim loads and the robots are visible.

### Step 3: Launch SLAM (Mapping)

In **Terminal 2**:
```bash
cd ~/ResQoUnity/go2_omniverse_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# For robot0
ros2 launch go2_navigation slam_launch.py robot_name:=robot0

# For robot1 (if you have multiple robots)
# ros2 launch go2_navigation slam_launch.py robot_name:=robot1
```

### Step 4: Visualize in RViz2

In **Terminal 3**:
```bash
source /opt/ros/humble/setup.bash
rviz2
```

**In RViz2:**
1. Set `Fixed Frame` to `map` (top left)
2. Click `Add` → `By topic` → Add:
   - `/map` (Map) - Shows SLAM map
   - `/robot0/scan` (LaserScan) - Shows lidar data
   - `/robot0/odom` (Odometry) - Shows robot pose
   - `/tf` (TF) - Shows coordinate frames

### Step 5: Drive the Robot to Build Map

You can control the robot using keyboard controls already in your simulation:
- **W/S**: Forward/Backward
- **A/D**: Left/Right rotation
- **Q/E**: Strafe left/right

As the robot moves, SLAM Toolbox will build the map in real-time!

### Step 6: Save the Map

Once you're happy with the map:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ResQoUnity/go2_omniverse_ws/src/go2_navigation/maps/my_map
```

This saves `my_map.yaml` and `my_map.pgm` files.

## 🗺️ Using Navigation (After Mapping)

### Option 1: Navigation with Existing Map

```bash
cd ~/ResQoUnity/go2_omniverse_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch go2_navigation navigation_launch.py \
    robot_name:=robot0 \
    map:=~/ResQoUnity/go2_omniverse_ws/src/go2_navigation/maps/my_map.yaml
```

### Option 2: Simultaneous SLAM and Navigation

For exploring unknown environments:
```bash
ros2 launch go2_navigation full_navigation_launch.py robot_name:=robot0
```

## 🎮 Sending Navigation Goals

### Using RViz2

1. Open RViz2 with navigation visualization
2. Click `Add` → `By display type` → `Nav2 Panel`
3. Click "2D Pose Estimate" to set robot's initial position
4. Click "Nav2 Goal" to send navigation goals

### Using Command Line

```bash
# Send a goal to (x=2.0, y=1.0, yaw=0.0)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

### Using Python Script

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

rclpy.init()
node = rclpy.create_node('goal_publisher')
pub = node.create_publisher(PoseStamped, '/goal_pose', 10)

goal = PoseStamped()
goal.header.frame_id = 'map'
goal.pose.position.x = 2.0
goal.pose.position.y = 1.0
goal.pose.orientation.w = 1.0

pub.publish(goal)
print("Goal sent!")
```

## 🔧 Troubleshooting

### Issue: "No transform from base_link to map"

**Solution**: Make sure SLAM is running and the robot has moved enough to initialize the map.

### Issue: "Pointcloud not converting to LaserScan"

**Solution**: Check that your simulation is publishing to `/robot0/point_cloud2`:
```bash
ros2 topic list | grep point_cloud
ros2 topic echo /robot0/point_cloud2 --once
```

### Issue: Robot not moving in Nav2

**Solution**: Verify cmd_vel is being subscribed:
```bash
ros2 topic info /robot0/cmd_vel
# Should show subscribers from your simulation
```

### Issue: SLAM map quality is poor

**Solution**: Adjust parameters in `config/slam_toolbox_params.yaml`:
- Increase `minimum_travel_distance` for slower mapping
- Adjust `correlation_search_space_dimension` for better loop closure
- Modify `resolution` for finer/coarser maps

## 📊 Topic Summary

### Published by Isaac Sim:
- `/robot{N}/point_cloud2` - LiDAR point cloud
- `/robot{N}/odom` - Odometry data
- `/robot{N}/imu` - IMU data
- `/robot{N}/joint_states` - Joint positions

### Published by Navigation Stack:
- `/robot{N}/scan` - Converted 2D laser scan
- `/map` - SLAM-generated map
- `/robot{N}/cmd_vel` - Velocity commands (consumed by Isaac Sim)

## 🔗 Multi-Robot Navigation

For multiple robots, launch separate instances:

**Terminal 1**: Isaac Sim with 2 robots
```bash
./start_simulation.sh --robot_amount 2
```

**Terminal 2**: SLAM for robot0
```bash
ros2 launch go2_navigation slam_launch.py robot_name:=robot0
```

**Terminal 3**: SLAM for robot1
```bash
ros2 launch go2_navigation slam_launch.py robot_name:=robot1
```

## 📚 Additional Resources

- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Isaac Sim ROS2 Bridge](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/index.html)

## 🐛 Known Limitations

1. **Ground truth odometry**: Currently using perfect odometry from simulation. In real scenarios, you'd need sensor fusion.
2. **Flat terrain assumption**: Nav2 works best on relatively flat terrain. Quadruped-specific planners may improve performance.
3. **2D navigation**: This setup uses 2D navigation. For 3D obstacle avoidance, consider additional packages.

## 📝 Configuration Files

- `config/slam_toolbox_params.yaml` - SLAM Toolbox parameters
- `config/nav2_params.yaml` - Nav2 stack parameters
- `launch/slam_launch.py` - SLAM + pointcloud conversion
- `launch/navigation_launch.py` - Full Nav2 stack
- `launch/full_navigation_launch.py` - Combined SLAM + Navigation

## 🎓 Learning Path

1. **Week 1**: Get familiar with SLAM mapping
   - Drive robot manually and build maps
   - Experiment with different environments
   - Save and load maps

2. **Week 2**: Practice autonomous navigation
   - Send simple navigation goals
   - Test obstacle avoidance
   - Tune navigation parameters

3. **Week 3**: Advanced features
   - Multi-waypoint navigation
   - Multi-robot coordination
   - Custom behavior trees

---

**Questions or Issues?** Check the main project README or open an issue!

