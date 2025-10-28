#!/bin/bash

# Run Isaac Sim with Crazyflie Quadcopter Drone
# This script launches the simulation with the drone as the primary agent

# Set default parameters
ROBOT="drone"
NUM_ROBOTS=1
TERRAIN="flat"
CUSTOM_ENV="office"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --num_robots)
      NUM_ROBOTS="$2"
      shift 2
      ;;
    --terrain)
      TERRAIN="$2"
      shift 2
      ;;
    --env)
      CUSTOM_ENV="$2"
      shift 2
      ;;
    --help)
      echo "Usage: ./run_sim_drone.sh [options]"
      echo ""
      echo "Options:"
      echo "  --num_robots N    Number of drones to spawn (default: 1)"
      echo "  --terrain TYPE    Terrain type: flat or rough (default: flat)"
      echo "  --env ENV         Environment: office or warehouse (default: office)"
      echo "  --help            Show this help message"
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      echo "Use --help for usage information"
      exit 1
      ;;
  esac
done

echo "=========================================="
echo "ResQoUnity - Drone Simulation"
echo "=========================================="
echo "Robot Type:    $ROBOT"
echo "Number:        $NUM_ROBOTS"
echo "Terrain:       $TERRAIN"
echo "Environment:   $CUSTOM_ENV"
echo "=========================================="
echo ""
echo "Controls:"
echo "  W/S - Forward/Backward"
echo "  A/D - Strafe Left/Right"
echo "  Q/E - Rotate Left/Right (Yaw)"
echo "  T/G - Altitude Up/Down"
echo ""
echo "Starting simulation..."
echo ""

# Run the simulation
python3 main.py \
    --robot $ROBOT \
    --robot_amount $NUM_ROBOTS \
    --terrain $TERRAIN \
    --custom_env $CUSTOM_ENV \
    --headless False

echo ""
echo "Simulation ended."

