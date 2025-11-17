#!/usr/bin/env python3
"""
Drone Flight Controller with Multiple Modes

Implements velocity, position, altitude hold, loiter, landing, and emergency modes
for autonomous drone control with PID-based position tracking.

Copyright (c) 2024, RoboVerse community
"""

from enum import Enum
import numpy as np
import time


class DroneState(Enum):
    """Flight modes for drone operation"""
    DISARMED = "DISARMED"           # Motors off, no commands accepted
    IDLE = "IDLE"                   # Armed, on ground, ready for takeoff
    VELOCITY = "VELOCITY"           # Direct velocity control via cmd_vel
    POSITION = "POSITION"           # Position setpoint tracking with PID
    ALTITUDE_HOLD = "ALTITUDE_HOLD" # Maintain altitude, XY velocity control
    LOITER = "LOITER"              # Hold current position (hover)
    LANDING = "LANDING"            # Controlled descent to ground
    EMERGENCY = "EMERGENCY"        # Emergency stop, all velocities zeroed


class PIDController:
    """
    PID controller for position/altitude tracking.
    
    Args:
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain
        max_output: Maximum output value (saturation limit)
        max_integral: Maximum integral windup value
    """
    
    def __init__(self, kp, ki, kd, max_output, max_integral=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.max_integral = max_integral
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
    
    def reset(self):
        """Reset PID state"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
    
    def update(self, error, dt):
        """
        Update PID controller with new error.
        
        Args:
            error: Current error (target - actual)
            dt: Time step in seconds
            
        Returns:
            Control output (velocity command)
        """
        if dt <= 0:
            return 0.0
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.max_integral, self.max_integral)
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        
        # Compute output with saturation
        output = p_term + i_term + d_term
        output = np.clip(output, -self.max_output, self.max_output)
        
        # Update state
        self.prev_error = error
        
        return output


class DroneController:
    """
    Main drone controller managing flight modes and position tracking.
    
    This controller implements multiple flight modes:
    - VELOCITY: Direct velocity commands from cmd_vel
    - POSITION: Automatic position tracking using PID control
    - ALTITUDE_HOLD: Maintain altitude while allowing XY velocity
    - LOITER: Hold current position (hover in place)
    - LANDING: Controlled descent to ground
    - EMERGENCY: Immediate stop
    
    Args:
        robot_id: Unique identifier for this drone (e.g., "0", "1")
        kp_pos: Proportional gain for XY position control
        ki_pos: Integral gain for XY position control
        kd_pos: Derivative gain for XY position control
        kp_alt: Proportional gain for altitude control
        ki_alt: Integral gain for altitude control
        kd_alt: Derivative gain for altitude control
        max_vel: Maximum horizontal velocity (m/s)
        max_climb_rate: Maximum vertical velocity (m/s)
    """
    
    def __init__(
        self,
        robot_id,
        kp_pos=1.0,
        ki_pos=0.01,
        kd_pos=0.5,
        kp_alt=2.0,
        ki_alt=0.1,
        kd_alt=1.0,
        kp_att=2.0,
        ki_att=0.0,
        kd_att=0.5,
        kp_yaw=1.0,
        ki_yaw=0.0,
        kd_yaw=0.2,
        max_vel=3.0,
        max_climb_rate=2.0,
        max_angle_rate=2.0,
        max_yaw_rate=1.0,
        hover_thrust=0.55
    ):
        self.robot_id = robot_id
        
        # Flight state
        self.mode = DroneState.DISARMED
        self.armed = False
        
        # Position tracking
        self.current_position = np.array([0.0, 0.0, 0.0])  # [x, y, z]
        self.current_velocity = np.array([0.0, 0.0, 0.0])  # [vx, vy, vz]
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.target_altitude = 0.0
        
        # Attitude tracking
        self.current_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # quaternion [w,x,y,z]
        self.current_euler = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw]
        
        # PID controllers for position (outer loop)
        self.pid_x = PIDController(kp_pos, ki_pos, kd_pos, max_vel)
        self.pid_y = PIDController(kp_pos, ki_pos, kd_pos, max_vel)
        self.pid_z = PIDController(kp_alt, ki_alt, kd_alt, max_climb_rate)
        
        # PID controllers for attitude (inner loop)
        self.pid_roll = PIDController(kp_att, ki_att, kd_att, max_angle_rate)
        self.pid_pitch = PIDController(kp_att, ki_att, kd_att, max_angle_rate)
        self.pid_yaw_rate = PIDController(kp_yaw, ki_yaw, kd_yaw, max_yaw_rate)
        
        # Velocity limits
        self.max_vel = max_vel
        self.max_climb_rate = max_climb_rate
        self.max_angle_rate = max_angle_rate
        self.max_yaw_rate = max_yaw_rate
        
        # Motor parameters
        self.motor_min = 0.0
        self.motor_max = 1.0
        self.hover_thrust = hover_thrust
        
        # Velocity command output (computed by controller)
        self.cmd_velocity = np.array([0.0, 0.0, 0.0, 0.0])  # [vx, vy, vz, yaw_rate]
        
        # Timing
        self.last_update_time = time.time()
        
        # Landing parameters
        self.landing_velocity = -0.3  # Descent rate during landing (m/s)
        self.landing_altitude_threshold = 0.1  # Auto-disarm below this height
        
        # Position tolerance for waypoint arrival
        self.position_tolerance = 0.5  # meters
    
    def set_mode(self, new_mode):
        """
        Transition to a new flight mode.
        
        Args:
            new_mode: DroneState or string name of new mode
        """
        if isinstance(new_mode, str):
            new_mode = DroneState[new_mode]
        
        # Mode transition logic
        if new_mode == self.mode:
            return  # Already in this mode
        
        # Reset PID controllers on mode change
        if new_mode in [DroneState.POSITION, DroneState.ALTITUDE_HOLD, DroneState.LOITER]:
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()
        
        # Special transitions
        if new_mode == DroneState.LOITER:
            # Capture current position as loiter target
            self.target_position = self.current_position.copy()
        
        if new_mode == DroneState.EMERGENCY:
            # Zero all velocities immediately
            self.cmd_velocity = np.array([0.0, 0.0, 0.0, 0.0])
        
        old_mode = self.mode
        self.mode = new_mode
        
        # Safety logging: Track unexpected mode changes
        if old_mode == DroneState.POSITION and new_mode == DroneState.LOITER:
            import traceback
            print(f"[SAFETY] Drone {self.robot_id} switched POSITION→LOITER (should only happen via service)")
            print(f"  Current: ({self.current_position[0]:.3f}, {self.current_position[1]:.3f}, {self.current_position[2]:.3f})")
            print(f"  Target:  ({self.target_position[0]:.3f}, {self.target_position[1]:.3f}, {self.target_position[2]:.3f})")
            print(f"  Call from: {traceback.format_stack()[-2].strip()}")
        
        print(f"[Drone {self.robot_id}] Mode: {old_mode.value} → {new_mode.value}")
    
    def set_target_position(self, x, y, z):
        """Set target position for POSITION mode"""
        self.target_position = np.array([x, y, z])
        print(f"[Drone {self.robot_id}] Target position: ({x:.2f}, {y:.2f}, {z:.2f})")
    
    def set_target_altitude(self, z):
        """Set target altitude for ALTITUDE_HOLD mode"""
        self.target_altitude = z
        print(f"[Drone {self.robot_id}] Target altitude: {z:.2f}m")
    
    def _quat_to_euler(self, quat):
        """
        Convert quaternion to Euler angles.
        
        Args:
            quat: Quaternion [w, x, y, z]
            
        Returns:
            np.array: Euler angles [roll, pitch, yaw] in radians
        """
        w, x, y, z = quat
        
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return np.array([roll, pitch, yaw])
    
    def _motor_mixer(self, thrust, roll_rate, pitch_rate, yaw_rate):
        """
        Convert thrust and attitude rates to individual motor commands.
        
        Crazyflie X-frame configuration:
            M1 (front-right)  M4 (front-left)
                   \  ^  /
                    \ | /
                      x
                    / | \
                   /  v  \
            M2 (rear-right)  M3 (rear-left)
        
        Args:
            thrust: Total thrust (0-1, normalized)
            roll_rate: Desired roll rate (rad/s), scaled
            pitch_rate: Desired pitch rate (rad/s), scaled
            yaw_rate: Desired yaw rate (rad/s), scaled
        
        Returns:
            np.array: [m1, m2, m3, m4] motor commands (0-1 each)
        """
        # Mixing matrix for X-frame
        # Each motor contributes to thrust, roll, pitch, yaw
        m1 = thrust + roll_rate - pitch_rate + yaw_rate   # Front-right
        m2 = thrust + roll_rate + pitch_rate - yaw_rate   # Rear-right
        m3 = thrust - roll_rate + pitch_rate + yaw_rate   # Rear-left
        m4 = thrust - roll_rate - pitch_rate - yaw_rate   # Front-left
        
        # Check saturation and scale down if needed
        motors = np.array([m1, m2, m3, m4])
        if np.any(motors > self.motor_max):
            # Scale down to prevent saturation
            scale = self.motor_max / np.max(motors)
            motors *= scale
        
        # Saturate motors to valid range
        motors = np.clip(motors, self.motor_min, self.motor_max)
        
        return motors
    
    def update(self, dt, current_pos, current_vel):
        """
        Update controller state and compute velocity commands.
        
        Args:
            dt: Time step since last update (seconds)
            current_pos: Current position [x, y, z] from simulator
            current_vel: Current velocity [vx, vy, vz] from simulator
        """
        self.current_position = np.array(current_pos)
        self.current_velocity = np.array(current_vel)
        
        # Auto-landing detection with ground contact
        if self.mode == DroneState.LANDING:
            if self.current_position[2] < self.landing_altitude_threshold and abs(self.current_velocity[2]) < 0.1:
                # Reached ground, disarm
                self.armed = False
                self.set_mode(DroneState.DISARMED)
                print(f"[Drone {self.robot_id}] Landed and disarmed")
                return
    
    def compute_motor_command(self):
        """
        Compute motor commands based on current flight mode.
        
        Returns:
            np.array: [m1, m2, m3, m4] motor commands (0-1 each)
        """
        if not self.armed or self.mode == DroneState.DISARMED:
            return np.zeros(4)
        
        if self.mode == DroneState.EMERGENCY:
            return np.zeros(4)
        
        if self.mode == DroneState.IDLE:
            # IDLE: motors at hover thrust to prevent falling
            # This allows the drone to hover while waiting for commands
            return self._motor_mixer(self.hover_thrust, 0.0, 0.0, 0.0)
        
        # Common: compute desired velocities first
        desired_vx = 0.0
        desired_vy = 0.0
        desired_vz = 0.0
        desired_yaw_rate = 0.0
        
        dt = 0.016  # 60 Hz
        
        if self.mode == DroneState.VELOCITY:
            # Velocities from external cmd_vel (handled in main loop)
            # Return hover thrust for now
            return self._motor_mixer(self.hover_thrust, 0.0, 0.0, 0.0)
        
        elif self.mode == DroneState.POSITION:
            # Position control: compute desired velocities
            error_x = self.target_position[0] - self.current_position[0]
            error_y = self.target_position[1] - self.current_position[1]
            error_z = self.target_position[2] - self.current_position[2]
            
            desired_vx = self.pid_x.update(error_x, dt)
            desired_vy = self.pid_y.update(error_y, dt)
            desired_vz = self.pid_z.update(error_z, dt)
            
            # Auto-LOITER removed - mode changes now controlled externally via services
        
        elif self.mode == DroneState.ALTITUDE_HOLD:
            # Altitude control only
            error_z = self.target_altitude - self.current_position[2]
            desired_vz = self.pid_z.update(error_z, dt)
            # XY from external commands (handled in main loop)
        
        elif self.mode == DroneState.LOITER:
            # Hold position
            error_x = self.target_position[0] - self.current_position[0]
            error_y = self.target_position[1] - self.current_position[1]
            error_z = self.target_position[2] - self.current_position[2]
            
            desired_vx = self.pid_x.update(error_x, dt)
            desired_vy = self.pid_y.update(error_y, dt)
            desired_vz = self.pid_z.update(error_z, dt)
        
        elif self.mode == DroneState.LANDING:
            desired_vz = self.landing_velocity
        
        # Convert desired velocities to attitude commands
        # For small angles: pitch ≈ -desired_vx, roll ≈ desired_vy
        # Reduced scaling to prevent aggressive tilting
        desired_pitch = -desired_vx * 0.15  # Reduced from 0.5
        desired_roll = desired_vy * 0.15    # Reduced from 0.5
        
        # Attitude control (inner loop)
        error_roll = desired_roll - self.current_euler[0]
        error_pitch = desired_pitch - self.current_euler[1]
        
        roll_rate = self.pid_roll.update(error_roll, dt)
        pitch_rate = self.pid_pitch.update(error_pitch, dt)
        
        # Thrust from vertical velocity
        # Hover thrust + correction from altitude error
        # Increased vertical gain for better altitude response
        thrust = self.hover_thrust + desired_vz * 0.2  # Increased from 0.1
        thrust = np.clip(thrust, 0.0, 1.0)
        
        # Yaw rate (keep zero or from external command)
        yaw_rate = desired_yaw_rate
        
        # Scale down attitude rates for motor mixer (prevent saturation)
        roll_rate_scaled = roll_rate * 0.05   # Scale to reasonable range
        pitch_rate_scaled = pitch_rate * 0.05
        yaw_rate_scaled = yaw_rate * 0.05
        
        # Motor mixing
        motors = self._motor_mixer(thrust, roll_rate_scaled, pitch_rate_scaled, yaw_rate_scaled)
        
        # Debug output
        if self.robot_id == "0":
            print(f"[PID] vel:({desired_vx:.2f},{desired_vy:.2f},{desired_vz:.2f}) thrust:{thrust:.3f} motors:[{motors[0]:.3f},{motors[1]:.3f},{motors[2]:.3f},{motors[3]:.3f}]")
        
        return motors

