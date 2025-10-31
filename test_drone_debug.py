#!/usr/bin/env python3
"""
Debug script to check drone state and control
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, Trigger
import sys

class DroneDebugger(Node):
    def __init__(self):
        super().__init__('drone_debugger')
        self.odom_sub = self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            10
        )
        self.odom_received = False
        self.last_position = None
        
    def odom_callback(self, msg):
        self.odom_received = True
        self.last_position = msg.pose.pose.position
        print(f"[DEBUG] Drone at: x={self.last_position.x:.2f}, "
              f"y={self.last_position.y:.2f}, z={self.last_position.z:.2f}")

def main():
    rclpy.init()
    node = DroneDebugger()
    
    print("=" * 60)
    print("DRONE DIAGNOSTICS")
    print("=" * 60)
    
    # Test 1: Check if odometry is publishing
    print("\n[TEST 1] Checking if drone odometry is publishing...")
    for i in range(20):  # Wait 2 seconds
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom_received:
            print(f"✅ Odometry WORKING - Drone position: "
                  f"({node.last_position.x:.2f}, {node.last_position.y:.2f}, {node.last_position.z:.2f})")
            break
    else:
        print("❌ Odometry NOT publishing - Drone view not initialized!")
        rclpy.shutdown()
        return
    
    # Test 2: Check current altitude
    print(f"\n[TEST 2] Current altitude: {node.last_position.z:.2f}m")
    if node.last_position.z < 0.5:
        print("⚠️  Drone is on the ground (z < 0.5m)")
    elif node.last_position.z > 2.0:
        print(f"✅ Drone is airborne at {node.last_position.z:.2f}m")
    else:
        print(f"🔄 Drone is at {node.last_position.z:.2f}m (climbing?)")
    
    # Test 3: Try to arm
    print("\n[TEST 3] Attempting to arm drone...")
    arm_client = node.create_client(SetBool, '/drone/arm')
    if not arm_client.wait_for_service(timeout_sec=2.0):
        print("❌ Arm service not available!")
        rclpy.shutdown()
        return
    
    req = SetBool.Request()
    req.data = True
    future = arm_client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    
    if future.result():
        print(f"✅ Arm response: {future.result().message}")
    else:
        print("❌ Arm service failed!")
        rclpy.shutdown()
        return
    
    # Test 4: Try takeoff
    print("\n[TEST 4] Attempting takeoff...")
    takeoff_client = node.create_client(Trigger, '/drone/takeoff')
    if not takeoff_client.wait_for_service(timeout_sec=2.0):
        print("❌ Takeoff service not available!")
        rclpy.shutdown()
        return
    
    req = Trigger.Request()
    future = takeoff_client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    
    if future.result():
        print(f"✅ Takeoff response: {future.result().message}")
    else:
        print("❌ Takeoff service failed!")
        rclpy.shutdown()
        return
    
    # Test 5: Monitor altitude for 5 seconds
    print("\n[TEST 5] Monitoring altitude for 5 seconds...")
    print("(Watch for altitude change - should climb to ~1.5m)")
    
    initial_z = node.last_position.z
    for i in range(50):  # 5 seconds
        rclpy.spin_once(node, timeout_sec=0.1)
        if i % 10 == 0:  # Print every second
            delta_z = node.last_position.z - initial_z
            print(f"  t={i/10:.1f}s: z={node.last_position.z:.2f}m (Δz={delta_z:+.2f}m)")
    
    final_z = node.last_position.z
    altitude_change = final_z - initial_z
    
    print(f"\n[RESULT] Altitude change: {altitude_change:+.2f}m")
    if abs(altitude_change) < 0.1:
        print("❌ DRONE NOT MOVING - Motors not responding!")
        print("\nPossible causes:")
        print("  1. Controller not receiving position data")
        print("  2. Motor commands not reaching ArticulationView")
        print("  3. Physics properties not set correctly")
    elif altitude_change > 0.5:
        print(f"✅ DRONE IS FLYING - Successfully climbed {altitude_change:.2f}m!")
    else:
        print(f"⚠️  DRONE MOVED SLIGHTLY - Only {altitude_change:.2f}m (check PID gains)")
    
    print("\n" + "=" * 60)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

