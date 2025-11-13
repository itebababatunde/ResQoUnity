#!/usr/bin/env python3
"""
Test script to verify drone force application is working correctly.

This script checks if the drone can:
1. Arm successfully
2. Generate thrust when armed
3. Maintain hover
4. Respond to position commands

Usage:
    Terminal 1: cd ~/ResQoUnity && ./start_simulation.sh go2 1 flat office
    Terminal 2: python3 test_drone_forces.py
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import PoseStamped
import time
import numpy as np


class DroneForceTest(Node):
    def __init__(self):
        super().__init__('drone_force_test')
        
        # Subscribe to odometry to monitor drone state
        self.odom_sub = self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            10
        )
        
        # Service clients
        self.arm_client = self.create_client(SetBool, '/drone/arm')
        self.takeoff_client = self.create_client(Trigger, '/drone/takeoff')
        self.land_client = self.create_client(Trigger, '/drone/land')
        
        # Position command publisher
        self.cmd_pos_pub = self.create_publisher(PoseStamped, '/drone/cmd_position', 10)
        
        # State tracking
        self.position_history = []
        self.last_position = None
        self.odom_received = False
        
        self.get_logger().info('Drone Force Test initialized')
    
    def odom_callback(self, msg):
        self.odom_received = True
        self.last_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        self.position_history.append((time.time(), self.last_position.copy()))
        
        # Keep only last 100 samples
        if len(self.position_history) > 100:
            self.position_history.pop(0)
    
    def call_arm(self, arm):
        """Arm or disarm the drone"""
        request = SetBool.Request()
        request.data = arm
        
        self.get_logger().info(f'Calling arm service: {arm}')
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().success
        return False
    
    def call_takeoff(self):
        """Call takeoff service"""
        request = Trigger.Request()
        future = self.takeoff_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().success
        return False
    
    def call_land(self):
        """Call land service"""
        request = Trigger.Request()
        future = self.land_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().success
        return False
    
    def send_position_cmd(self, x, y, z):
        """Send position command"""
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.cmd_pos_pub.publish(msg)
    
    def check_altitude_change(self, duration=5.0):
        """Check if altitude changes over duration"""
        self.position_history.clear()
        
        # Collect data
        start_time = time.time()
        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if len(self.position_history) < 2:
            return False, 0.0
        
        # Calculate altitude change
        initial_z = self.position_history[0][1][2]
        final_z = self.position_history[-1][1][2]
        delta_z = final_z - initial_z
        
        return abs(delta_z) > 0.05, delta_z  # Moved more than 5cm
    
    def check_hovering_stability(self, duration=5.0, tolerance=0.2):
        """Check if drone maintains altitude with minimal drift"""
        self.position_history.clear()
        
        # Collect data
        start_time = time.time()
        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if len(self.position_history) < 10:
            return False, 0.0
        
        # Calculate altitude variance
        altitudes = [pos[1][2] for pos in self.position_history]
        mean_alt = np.mean(altitudes)
        std_alt = np.std(altitudes)
        
        return std_alt < tolerance, std_alt


def main(args=None):
    rclpy.init(args=args)
    
    tester = DroneForceTest()
    
    print("\n" + "="*70)
    print("DRONE FORCE APPLICATION TEST")
    print("="*70)
    print("\nThis test verifies that drone forces are being applied correctly")
    print("Make sure simulation is running:")
    print("  cd ~/ResQoUnity && ./start_simulation.sh go2 1 flat office")
    print("\nPress Enter to begin test...")
    input()
    
    try:
        # Test 1: Check odometry is publishing
        print("\n[TEST 1] Checking odometry...")
        for i in range(20):
            rclpy.spin_once(tester, timeout_sec=0.1)
            if tester.odom_received:
                print(f"✅ Odometry working - Drone at: {tester.last_position}")
                break
        else:
            print("❌ FAIL: No odometry received")
            return
        
        initial_altitude = tester.last_position[2]
        
        # Test 2: Arm the drone
        print("\n[TEST 2] Arming drone...")
        if not tester.call_arm(True):
            print("❌ FAIL: Could not arm drone")
            return
        print("✅ Drone armed")
        time.sleep(1)
        
        # Test 3: Check if drone is falling or stable when armed
        print("\n[TEST 3] Checking if drone maintains altitude when armed (5s)...")
        moved, delta_z = tester.check_altitude_change(duration=5.0)
        
        if delta_z < -0.5:
            print(f"❌ FAIL: Drone is falling! Δz = {delta_z:.2f}m")
            print("   This indicates forces are NOT being applied!")
        elif abs(delta_z) < 0.1:
            print(f"✅ PASS: Drone is stable, Δz = {delta_z:.2f}m")
        else:
            print(f"⚠️  WARNING: Drone drifted {delta_z:.2f}m (may need PID tuning)")
        
        # Test 4: Takeoff command
        print("\n[TEST 4] Testing takeoff...")
        if not tester.call_takeoff():
            print("❌ FAIL: Takeoff command failed")
            return
        print("⏳ Waiting 10 seconds for takeoff...")
        time.sleep(10)
        
        # Test 5: Check altitude increased
        print("\n[TEST 5] Checking if drone climbed...")
        current_altitude = tester.last_position[2]
        altitude_gain = current_altitude - initial_altitude
        
        if altitude_gain > 0.5:
            print(f"✅ PASS: Drone climbed {altitude_gain:.2f}m!")
            print("   Forces are being applied correctly!")
        elif altitude_gain > 0.1:
            print(f"⚠️  PARTIAL: Drone climbed {altitude_gain:.2f}m (may need more thrust)")
        else:
            print(f"❌ FAIL: Drone did not climb (Δz = {altitude_gain:.2f}m)")
            print("   Forces may not be applied or thrust is too weak")
        
        # Test 6: Hover stability
        print("\n[TEST 6] Testing hover stability (5s)...")
        stable, std_alt = tester.check_hovering_stability(duration=5.0)
        
        if stable:
            print(f"✅ PASS: Hover is stable (σ = {std_alt:.3f}m)")
        else:
            print(f"⚠️  WARNING: Hover is unstable (σ = {std_alt:.3f}m)")
            print("   May need PID tuning")
        
        # Test 7: Position command response
        print("\n[TEST 7] Testing position control...")
        target_pos = [2.0, 1.0, 3.0]
        print(f"   Sending command to: {target_pos}")
        tester.send_position_cmd(*target_pos)
        
        print("⏳ Waiting 15 seconds for drone to reach target...")
        time.sleep(15)
        
        final_pos = tester.last_position
        error = np.linalg.norm(final_pos - np.array(target_pos))
        
        if error < 0.5:
            print(f"✅ PASS: Reached target (error = {error:.2f}m)")
        elif error < 1.0:
            print(f"⚠️  PARTIAL: Close to target (error = {error:.2f}m)")
        else:
            print(f"❌ FAIL: Did not reach target (error = {error:.2f}m)")
        
        # Test 8: Landing
        print("\n[TEST 8] Testing landing...")
        if not tester.call_land():
            print("❌ FAIL: Land command failed")
        else:
            print("⏳ Waiting 10 seconds for landing...")
            time.sleep(10)
            
            final_altitude = tester.last_position[2]
            if final_altitude < 0.5:
                print(f"✅ PASS: Drone landed (z = {final_altitude:.2f}m)")
            else:
                print(f"⚠️  WARNING: Drone still airborne (z = {final_altitude:.2f}m)")
        
        # Summary
        print("\n" + "="*70)
        print("TEST SUMMARY")
        print("="*70)
        print(f"Initial altitude: {initial_altitude:.2f}m")
        print(f"Max altitude: {current_altitude:.2f}m")
        print(f"Altitude gain: {altitude_gain:.2f}m")
        print(f"Final altitude: {tester.last_position[2]:.2f}m")
        
        if altitude_gain > 0.5:
            print("\n✅ OVERALL: Drone control is WORKING!")
            print("   Forces are being applied correctly")
        elif altitude_gain > 0.1:
            print("\n⚠️  OVERALL: Drone control PARTIALLY working")
            print("   May need thrust coefficient tuning")
        else:
            print("\n❌ OVERALL: Drone control NOT working")
            print("   Check simulation console for force application errors")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    finally:
        tester.destroy_node()
        rclpy.shutdown()
        print("Test complete")


if __name__ == '__main__':
    main()

