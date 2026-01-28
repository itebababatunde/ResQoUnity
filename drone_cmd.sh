#!/bin/bash
# Drone Control Script - Simple commands for the world drone
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

case "$1" in
  arm)     ros2 service call /drone/arm std_srvs/srv/SetBool "{data: true}" ;;
  disarm)  ros2 service call /drone/arm std_srvs/srv/SetBool "{data: false}" ;;
  takeoff) ros2 service call /drone/takeoff std_srvs/srv/Trigger "{}" ;;
  land)    ros2 service call /drone/land std_srvs/srv/Trigger "{}" ;;
  stop)    ros2 service call /drone/emergency_stop std_srvs/srv/Trigger "{}" ;;
  status)  ros2 topic echo /drone/odom --once ;;
  *)       echo "Usage: ./drone_cmd.sh [arm|disarm|takeoff|land|stop|status]" ;;
esac
