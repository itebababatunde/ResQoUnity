# Drone SLAM Guide

## Overview

The drone integration **fully supports SLAM** (Simultaneous Localization and Mapping). This guide explains how to use SLAM with drones for aerial mapping and navigation.

## Quick Start

### Option 1: Drone-Optimized SLAM (Recommended)

```bash
# Terminal 1: Launch drone simulation
./run_sim_drone.sh

# Terminal 2: Launch drone SLAM
ros2 launch go2_navigation drone_slam_launch.py \
  robot_name:=robot0 \
  flight_altitude:=2.0 \
  use_sim_time:=false
```

### Option 2: Standard SLAM (Works but not optimized)

```bash
# Terminal 1: Launch drone simulation
./run_sim_drone.sh

# Terminal 2: Launch standard SLAM
ros2 launch go2_navigation slam_launch.py \
  robot_name:=robot0 \
  use_sim_time:=false
```

## Understanding Aerial SLAM

### 2D vs 3D SLAM

**2D SLAM (Current Implementation):**
- Creates a horizontal slice through the point cloud at drone altitude
- Generates a 2D occupancy grid map
- Lighter weight, faster processing
- Good for indoor environments with vertical walls

**3D SLAM (Future Enhancement):**
- Uses full 3D point cloud
- Creates volumetric maps
- More accurate for complex environments
- Requires more computational resources

### How Drone SLAM Works

```
┌─────────────────────────────────────────┐
│  Drone Flight Altitude: 2.0m            │
├─────────────────────────────────────────┤
│                                         │
│  Point Cloud (360° lidar)               │
│         ↓                               │
│  Height Filtering                       │
│  (slice at altitude ± thickness)        │
│         ↓                               │
│  2D LaserScan                          │
│         ↓                               │
│  SLAM Toolbox                          │
│         ↓                               │
│  2D Occupancy Grid Map                 │
└─────────────────────────────────────────┘
```

## Launch Parameters

### `drone_slam_launch.py` Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_name` | `robot0` | Robot namespace |
| `flight_altitude` | `2.0` | Drone altitude in meters |
| `slice_thickness` | `1.0` | Thickness of horizontal slice |
| `use_sim_time` | `false` | Use simulation time |

### Example: Different Altitudes

```bash
# Low altitude mapping (1 meter)
ros2 launch go2_navigation drone_slam_launch.py \
  robot_name:=robot0 \
  flight_altitude:=1.0 \
  slice_thickness:=0.5

# High altitude mapping (3 meters)
ros2 launch go2_navigation drone_slam_launch.py \
  robot_name:=robot0 \
  flight_altitude:=3.0 \
  slice_thickness:=1.5
```

## Visualizing Maps in RViz2

### Launch RViz2

```bash
rviz2
```

### Configure RViz2 for Drone SLAM

1. **Set Fixed Frame:** `map`

2. **Add Display - Map:**
   - Type: `Map`
   - Topic: `/map`
   - Color Scheme: `map` or `costmap`

3. **Add Display - LaserScan:**
   - Type: `LaserScan`
   - Topic: `/robot0/scan`
   - Size: `0.05`
   - Color: Red

4. **Add Display - Robot Model (optional):**
   - Type: `TF`
   - Frames: Show `odom`, `map`, `robot0/base_link`

5. **Add Display - Path (optional):**
   - Type: `Path`
   - Topic: `/trajectory` (if available)

### RViz2 Configuration File

Save this configuration to `drone_slam.rviz`:

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      
    - Class: rviz_default_plugins/Map
      Name: Map
      Topic: /map
      
    - Class: rviz_default_plugins/LaserScan
      Name: Scan
      Topic: /robot0/scan
      Size (m): 0.05
      Color: 255; 0; 0
      
    - Class: rviz_default_plugins/TF
      Name: TF
      
  Global Options:
    Fixed Frame: map
    
  Views:
    - Class: rviz_default_plugins/TopDownOrtho
      Name: TopDown
      Scale: 20.0
```

Load with:
```bash
rviz2 -d drone_slam.rviz
```

## Topics Published by Drone SLAM

| Topic | Type | Description |
|-------|------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | 2D occupancy grid map |
| `/robot0/scan` | `sensor_msgs/LaserScan` | 2D laser scan from point cloud |
| `/slam_toolbox/graph_visualization` | `visualization_msgs/MarkerArray` | SLAM graph |
| `/slam_toolbox/scan_visualization` | `sensor_msgs/LaserScan` | Matched scans |

## Saving Maps

### Save Map During Flight

```bash
# In a new terminal while SLAM is running
ros2 run nav2_map_server map_saver_cli -f my_drone_map
```

This creates two files:
- `my_drone_map.yaml` - Map metadata
- `my_drone_map.pgm` - Map image

### Map File Structure

`my_drone_map.yaml`:
```yaml
image: my_drone_map.pgm
resolution: 0.05
origin: [-10.0, -10.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

## Using Maps for Navigation

Once you have a map, use it for autonomous navigation:

```bash
# Terminal 1: Launch drone with saved map
./run_sim_drone.sh

# Terminal 2: Launch navigation with map
ros2 launch go2_navigation navigation_launch.py \
  robot_name:=robot0 \
  map:=/path/to/my_drone_map.yaml
```

## Multi-Drone SLAM

### Separate Maps per Drone

```bash
# Drone 1
ros2 launch go2_navigation drone_slam_launch.py \
  robot_name:=robot0 \
  flight_altitude:=2.0

# Drone 2
ros2 launch go2_navigation drone_slam_launch.py \
  robot_name:=robot1 \
  flight_altitude:=2.5
```

### Collaborative SLAM (Advanced)

For multi-drone collaborative mapping, you'll need additional tools:
- `slam_toolbox` multi-robot support
- Map merging algorithms
- Shared map frame coordination

## Optimization Tips

### For Better Maps

1. **Fly Smoothly:**
   - Avoid sudden movements
   - Maintain consistent altitude
   - Slower is better for mapping

2. **Environment Considerations:**
   - Best with vertical structures (walls, columns)
   - Challenging in open spaces
   - Good lighting helps camera (if used)

3. **Adjust Parameters:**
   ```bash
   # For dense environments (many obstacles)
   slice_thickness:=0.5  # Thinner slice
   
   # For sparse environments (few obstacles)
   slice_thickness:=1.5  # Thicker slice
   ```

### SLAM Toolbox Parameters

Edit `config/slam_toolbox_params.yaml` for tuning:

```yaml
# Increase for faster mapping (less accurate)
minimum_travel_distance: 0.5

# Decrease for better loop closures
loop_search_maximum_distance: 5.0

# Increase for larger maps
max_laser_range: 30.0
```

## Troubleshooting

### Map Not Updating

**Problem:** SLAM map shows blank or doesn't update

**Solutions:**
1. Check drone is publishing point cloud:
   ```bash
   ros2 topic hz /robot0/point_cloud2
   ```

2. Verify scan conversion is working:
   ```bash
   ros2 topic echo /robot0/scan --once
   ```

3. Check TF tree:
   ```bash
   ros2 run tf2_tools view_frames
   ```

### Poor Map Quality

**Problem:** Map is noisy or inaccurate

**Solutions:**
1. Adjust flight altitude to match environment
2. Increase slice thickness for more data
3. Fly slower for better scan matching
4. Check lidar orientation and mounting

### LaserScan Shows No Data

**Problem:** `/robot0/scan` topic is empty

**Solutions:**
1. Verify point cloud has data:
   ```bash
   ros2 topic echo /robot0/point_cloud2 --once
   ```

2. Check height filter parameters match altitude
3. Ensure `min_height` < `max_height`
4. Increase `slice_thickness` to capture more points

### TF Transform Errors

**Problem:** "Could not transform" errors in logs

**Solutions:**
1. Ensure odometry is publishing:
   ```bash
   ros2 topic hz /robot0/odom
   ```

2. Check TF tree is complete:
   ```bash
   ros2 run tf2_ros tf2_echo map robot0/base_link
   ```

3. Verify base_frame parameter matches robot namespace

## Performance Considerations

### Computational Load

Aerial SLAM is resource-intensive:

| Component | CPU Usage | Notes |
|-----------|-----------|-------|
| Point Cloud | Medium | 3D lidar data |
| PC to Laser | Low | Conversion is fast |
| SLAM Toolbox | High | Graph optimization |
| Map Updates | Medium | Grid updates |

### Optimization Strategies

1. **Reduce Update Rate:**
   ```yaml
   minimum_time_interval: 0.3  # Slower updates
   ```

2. **Limit Map Size:**
   ```yaml
   max_laser_range: 15.0  # Smaller range
   ```

3. **Disable Loop Closure (if needed):**
   ```yaml
   do_loop_closing: false
   ```

## Differences from Ground Robot SLAM

| Aspect | Ground Robot | Drone |
|--------|--------------|-------|
| **Height Filtering** | 0-2m from ground | Slice at altitude |
| **Movement** | 2D plane (X, Y) | 3D space (X, Y, Z) |
| **Stability** | Stable base | Flight dynamics |
| **Range** | Limited by obstacles | Aerial view, longer range |
| **Scan Pattern** | Horizontal at fixed height | Horizontal at flight altitude |

## Advanced: 3D SLAM (Future)

For full 3D mapping, consider:

### Alternative Tools

1. **RTAB-Map:**
   - Full 3D RGB-D SLAM
   - Uses camera + lidar
   - Heavier computation

2. **LIO-SAM:**
   - Lidar-Inertial odometry
   - Better for fast motion
   - Requires IMU integration

3. **Cartographer:**
   - Google's SLAM solution
   - Good for large spaces
   - 2D and 3D support

### Integration Example (RTAB-Map)

```bash
# Future integration
ros2 launch rtabmap_ros rtabmap.launch.py \
  frame_id:=robot0/base_link \
  rgb_topic:=/robot0/front_cam/rgb \
  depth_topic:=/robot0/depth \
  cloud_topic:=/robot0/point_cloud2
```

## Example Workflow

### Complete Mapping Session

```bash
# 1. Start drone simulation
./run_sim_drone.sh --terrain flat --env office

# 2. Launch SLAM
ros2 launch go2_navigation drone_slam_launch.py \
  robot_name:=robot0 \
  flight_altitude:=2.0

# 3. Start RViz2 for visualization
rviz2 -d drone_slam.rviz

# 4. Fly drone to map the area (using keyboard or cmd_vel)
# - Use WASD for movement
# - Use T/G for altitude adjustment
# - Fly systematically to cover area

# 5. Save the map
ros2 run nav2_map_server map_saver_cli -f office_aerial_map

# 6. Later: Use map for navigation
ros2 launch go2_navigation navigation_launch.py \
  robot_name:=robot0 \
  map:=$(pwd)/office_aerial_map.yaml
```

## Conclusion

✅ **Drone SLAM is fully supported!**

The drone integration includes:
- ✅ Point cloud publishing
- ✅ Odometry and IMU data
- ✅ TF transformations
- ✅ Optimized launch file for aerial SLAM
- ✅ Configurable altitude parameters

For best results:
1. Use `drone_slam_launch.py` for aerial-optimized SLAM
2. Adjust `flight_altitude` to match your environment
3. Fly smoothly and systematically
4. Visualize in RViz2 for real-time feedback

---

**See DRONE_README.md for general drone documentation**

