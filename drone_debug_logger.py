#!/usr/bin/env python3
"""
Comprehensive drone debugging logger.
Tracks all state changes, commands, and control outputs.
"""

import time
import numpy as np


class DroneDebugLogger:
    """Detailed logging for drone control debugging"""
    
    def __init__(self, robot_id="drone"):
        self.robot_id = robot_id
        self.frame_count = 0
        self.last_log_time = time.time()
        self.log_interval = 1.0  # Log every 1 second
        
        # State history
        self.position_history = []
        self.velocity_history = []
        self.command_history = []
        
        print(f"\n{'='*80}")
        print(f"DRONE DEBUG LOGGER INITIALIZED FOR {robot_id}")
        print(f"{'='*80}\n")
    
    def log_initialization(self, initial_pos, initial_vel, mode):
        """Log initial drone state"""
        print(f"[INIT] Drone spawned at position: ({initial_pos[0]:.3f}, {initial_pos[1]:.3f}, {initial_pos[2]:.3f})")
        print(f"[INIT] Initial velocity: ({initial_vel[0]:.3f}, {initial_vel[1]:.3f}, {initial_vel[2]:.3f})")
        print(f"[INIT] Initial mode: {mode}")
        print(f"[INIT] Ready for control\n")
    
    def log_arm_event(self, armed):
        """Log arming/disarming"""
        status = "ARMED" if armed else "DISARMED"
        print(f"\n{'*'*80}")
        print(f"[ARM EVENT] Drone {status}")
        print(f"{'*'*80}\n")
    
    def log_mode_change(self, old_mode, new_mode, target_pos=None):
        """Log mode transitions"""
        print(f"\n[MODE CHANGE] {old_mode} → {new_mode}")
        if target_pos is not None:
            print(f"[MODE CHANGE] Target position: ({target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f})")
        print()
    
    def log_frame(self, current_pos, current_vel, target_pos, mode, 
                  desired_vel, error, armed, actual_vel_applied=None, applied_force=None):
        """Log frame-by-frame state"""
        self.frame_count += 1
        current_time = time.time()
        
        # Log every second
        if current_time - self.last_log_time >= self.log_interval:
            self.last_log_time = current_time
            
            print(f"\n{'─'*80}")
            print(f"[FRAME {self.frame_count}] Δt = {self.log_interval:.2f}s")
            print(f"{'─'*80}")
            print(f"  State      : Armed={armed}, Mode={mode}")
            print(f"  Position   : ({current_pos[0]:7.3f}, {current_pos[1]:7.3f}, {current_pos[2]:7.3f})")
            print(f"  Velocity   : ({current_vel[0]:7.3f}, {current_vel[1]:7.3f}, {current_vel[2]:7.3f})")
            print(f"  Target Pos : ({target_pos[0]:7.3f}, {target_pos[1]:7.3f}, {target_pos[2]:7.3f})")
            print(f"  Error      : ({error[0]:7.3f}, {error[1]:7.3f}, {error[2]:7.3f}) = {np.linalg.norm(error):.3f}m")
            print(f"  PID Output : ({desired_vel[0]:7.3f}, {desired_vel[1]:7.3f}, {desired_vel[2]:7.3f}) m/s")
            
            if applied_force is not None:
                force_mag = np.linalg.norm(applied_force)
                print(f"  Applied Force: ({applied_force[0]:7.2f}, {applied_force[1]:7.2f}, {applied_force[2]:7.2f}) N (|F|={force_mag:.2f}N)")
                # Check if gravity is properly compensated (for LOITER mode)
                if armed and mode in ['LOITER', 'POSITION']:
                    # At hover, vertical force should be ~mass*g
                    # We don't have mass here, but can check if z-force is positive and reasonable
                    if applied_force[2] > 0:
                        print(f"  ✅ Gravity compensation active (Fz > 0)")
                    else:
                        print(f"  ⚠️  WARNING: No upward force (drone will fall!)")
            
            if actual_vel_applied is not None:
                print(f"  Actual Vel: ({actual_vel_applied[0]:7.3f}, {actual_vel_applied[1]:7.3f}, {actual_vel_applied[2]:7.3f}) m/s")
                vel_match = np.allclose(desired_vel, actual_vel_applied[:3], atol=0.01)
                if vel_match:
                    print(f"  ✅ Velocity matches PID output")
                else:
                    print(f"  ℹ️  Velocity differs from PID (physics is active)")
            
            # Check if drone is moving
            speed = np.linalg.norm(current_vel)
            if armed and speed < 0.01 and np.linalg.norm(error) > 0.5:
                print(f"  ⚠️  WARNING: Drone is STUCK (not moving toward target)")
            
            # Check altitude stability
            if mode in ['LOITER', 'POSITION']:
                altitude_error = abs(error[2])
                if altitude_error > 0.5:
                    print(f"  ⚠️  WARNING: Large altitude error ({altitude_error:.2f}m)")
            
            print(f"{'─'*80}\n")
    
    def log_pid_details(self, axis, error, p_term, i_term, d_term, output):
        """Log PID calculation details"""
        print(f"    PID[{axis}]: err={error:7.3f} | P={p_term:7.3f} | I={i_term:7.3f} | D={d_term:7.3f} → out={output:7.3f}")
    
    def log_velocity_application(self, desired_vel, method="set_velocities"):
        """Log when velocity command is sent"""
        print(f"  [APPLY] Sending velocity via {method}: ({desired_vel[0]:.3f}, {desired_vel[1]:.3f}, {desired_vel[2]:.3f})")
    
    def log_error(self, error_msg, exception=None):
        """Log errors"""
        print(f"\n{'!'*80}")
        print(f"[ERROR] {error_msg}")
        if exception:
            print(f"[ERROR] Exception: {exception}")
        print(f"{'!'*80}\n")
    
    def log_takeoff(self, current_alt, target_alt):
        """Log takeoff event"""
        print(f"\n[TAKEOFF] Current altitude: {current_alt:.2f}m → Target: {target_alt:.2f}m")
        print(f"[TAKEOFF] Need to climb: {target_alt - current_alt:.2f}m\n")
    
    def log_landing(self, current_alt):
        """Log landing event"""
        print(f"\n[LANDING] Starting descent from {current_alt:.2f}m")
        print(f"[LANDING] Will descend at {-0.3:.2f}m/s\n")
    
    def log_summary(self, start_pos, end_pos, duration):
        """Log session summary"""
        displacement = np.linalg.norm(end_pos - start_pos)
        altitude_change = end_pos[2] - start_pos[2]
        
        print(f"\n{'='*80}")
        print(f"DRONE CONTROL SESSION SUMMARY")
        print(f"{'='*80}")
        print(f"  Duration      : {duration:.1f}s")
        print(f"  Start Position: ({start_pos[0]:.2f}, {start_pos[1]:.2f}, {start_pos[2]:.2f})")
        print(f"  End Position  : ({end_pos[0]:.2f}, {end_pos[1]:.2f}, {end_pos[2]:.2f})")
        print(f"  Displacement  : {displacement:.2f}m")
        print(f"  Altitude Δ    : {altitude_change:+.2f}m")
        print(f"{'='*80}\n")

