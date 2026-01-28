# Drone Commands

Control the world drone using `./drone_cmd.sh`:

```bash
./drone_cmd.sh arm      # Arm the drone
./drone_cmd.sh disarm   # Disarm the drone
./drone_cmd.sh takeoff  # Takeoff
./drone_cmd.sh land     # Land
./drone_cmd.sh stop     # Emergency stop
./drone_cmd.sh status   # Show current position
```

## Quick Start

1. Start simulation: `./start_simulation.sh`
2. In another terminal:
   ```bash
   cd ~/ResQoUnity
   ./drone_cmd.sh arm
   ./drone_cmd.sh takeoff
   ```
