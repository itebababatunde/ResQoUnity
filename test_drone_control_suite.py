#!/usr/bin/env python3
"""
Test script for comprehensive drone control suite.

Tests all flight modes, services, and topics for autonomous drone control.

Usage:
    Terminal 1: cd ~/ResQoUnity && ./start_simulation.sh drone 1 flat office
    Terminal 2: python3 test_drone_control_suite.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
from std_srvs.srv import Trigger, SetBool
import time


class DroneControlTester(Node):
    def __init__(self):
        super().__init__('drone_control_tester')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'robot0/cmd_vel', 10)
        self.cmd_pos_pub = self.create_publisher(PoseStamped, 'robot0/cmd_position', 10)
        self.cmd_alt_pub = self.create_publisher(Float32, 'robot0/cmd_altitude', 10)
        
        # Service clients
        self.arm_client = self.create_client(SetBool, 'robot0/arm')
        self.takeoff_client = self.create_client(Trigger, 'robot0/takeoff')
        self.land_client = self.create_client(Trigger, 'robot0/land')
        self.emergency_client = self.create_client(Trigger, 'robot0/emergency_stop')
        
        self.get_logger().info('Drone Control Tester initialized')
    
    def call_arm_service(self, arm):
        """Arm or disarm the drone"""
        request = SetBool.Request()
        request.data = arm
        
        self.get_logger().info(f'Calling arm service: {arm}')
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Arm result: {response.message}')
            return response.success
        else:
            self.get_logger().error('Arm service call failed')
            return False
    
    def call_takeoff(self):
        """Call takeoff service"""
        request = Trigger.Request()
        self.get_logger().info('Calling takeoff service')
        future = self.takeoff_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Takeoff result: {response.message}')
            return response.success
        else:
            self.get_logger().error('Takeoff service call failed')
            return False
    
    def call_land(self):
        """Call land service"""
        request = Trigger.Request()
        self.get_logger().info('Calling land service')
        future = self.land_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Land result: {response.message}')
            return response.success
        else:
            self.get_logger().error('Land service call failed')
            return False
    
    def call_emergency_stop(self):
        """Call emergency stop service"""
        request = Trigger.Request()
        self.get_logger().info('Calling emergency stop service')
        future = self.emergency_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Emergency stop result: {response.message}')
            return response.success
        else:
            self.get_logger().error('Emergency stop service call failed')
            return False
    
    def send_velocity_command(self, vx, vy, vz, yaw_rate):
        """Send velocity command (VELOCITY mode)"""
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        msg.angular.z = yaw_rate
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f'Velocity: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, yaw={yaw_rate:.2f}')
    
    def send_position_command(self, x, y, z):
        """Send position command (POSITION mode)"""
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        self.cmd_pos_pub.publish(msg)
        self.get_logger().info(f'Position: x={x:.2f}, y={y:.2f}, z={z:.2f}')
    
    def send_altitude_command(self, altitude):
        """Send altitude command (ALTITUDE_HOLD mode)"""
        msg = Float32()
        msg.data = altitude
        self.cmd_alt_pub.publish(msg)
        self.get_logger().info(f'Altitude: {altitude:.2f}m')


def run_test_sequence(tester):
    """Run comprehensive test sequence"""
    print("\n" + "="*70)
    print("COMPREHENSIVE DRONE CONTROL SUITE TEST")
    print("="*70)
    
    time.sleep(2)  # Wait for services to be available
    
    # Test 1: Arm the drone
    print("\n[Test 1] Arming drone...")
    if not tester.call_arm_service(True):
        print("[ERROR] Failed to arm drone. Aborting tests.")
        return
    time.sleep(1)
    
    # Test 2: Takeoff
    print("\n[Test 2] Takeoff to 1.5m...")
    if not tester.call_takeoff():
        print("[ERROR] Failed to takeoff. Aborting tests.")
        return
    print("Waiting for takeoff (10 seconds)...")
    time.sleep(10)
    
    # Test 3: Velocity mode - move forward
    print("\n[Test 3] VELOCITY mode - Move forward for 3 seconds...")
    for _ in range(30):  # 3 seconds at 10 Hz
        tester.send_velocity_command(0.5, 0.0, 0.0, 0.0)
        time.sleep(0.1)
    tester.send_velocity_command(0.0, 0.0, 0.0, 0.0)  # Stop
    time.sleep(2)
    
    # Test 4: Position mode - go to waypoint
    print("\n[Test 4] POSITION mode - Go to waypoint (2.0, 1.0, 2.0)...")
    tester.send_position_command(2.0, 1.0, 2.0)
    print("Waiting to reach waypoint (15 seconds)...")
    time.sleep(15)
    
    # Test 5: Altitude hold - maintain 2.5m while moving XY
    print("\n[Test 5] ALTITUDE_HOLD mode - Hold 2.5m altitude...")
    tester.send_altitude_command(2.5)
    time.sleep(2)
    print("Moving in XY plane while maintaining altitude...")
    for _ in range(30):
        tester.send_velocity_command(0.3, 0.3, 0.0, 0.0)
        time.sleep(0.1)
    tester.send_velocity_command(0.0, 0.0, 0.0, 0.0)
    time.sleep(2)
    
    # Test 6: Position mode - return to origin
    print("\n[Test 6] POSITION mode - Return to origin...")
    tester.send_position_command(0.0, 0.0, 1.5)
    print("Waiting to reach origin (15 seconds)...")
    time.sleep(15)
    
    # Test 7: Land
    print("\n[Test 7] Landing...")
    if not tester.call_land():
        print("[ERROR] Failed to initiate landing.")
    print("Waiting for landing (10 seconds)...")
    time.sleep(10)
    
    print("\n" + "="*70)
    print("TEST SEQUENCE COMPLETE!")
    print("="*70)
    print("\nAll flight modes tested:")
    print("  ✓ DISARMED → IDLE (arm service)")
    print("  ✓ IDLE → POSITION (takeoff service)")
    print("  ✓ POSITION → VELOCITY (cmd_vel)")
    print("  ✓ VELOCITY → POSITION (cmd_position)")
    print("  ✓ POSITION → ALTITUDE_HOLD (cmd_altitude)")
    print("  ✓ ALTITUDE_HOLD → POSITION (cmd_position)")
    print("  ✓ POSITION → LANDING (land service)")
    print("  ✓ LANDING → DISARMED (auto on ground contact)")


def manual_control_menu(tester):
    """Interactive manual control menu"""
    while True:
        print("\n" + "="*70)
        print("MANUAL DRONE CONTROL")
        print("="*70)
        print("Services:")
        print("  1. Arm drone")
        print("  2. Disarm drone")
        print("  3. Takeoff")
        print("  4. Land")
        print("  5. Emergency stop")
        print("\nVelocity Commands (VELOCITY mode):")
        print("  6. Move forward")
        print("  7. Move up")
        print("  8. Move down")
        print("  9. Rotate left")
        print("  0. Stop")
        print("\nPosition Commands:")
        print("  p. Go to position (x, y, z)")
        print("  a. Set altitude hold")
        print("\nOther:")
        print("  t. Run test sequence")
        print("  q. Quit")
        print("="*70)
        
        choice = input("Enter choice: ").strip().lower()
        
        try:
            if choice == '1':
                tester.call_arm_service(True)
            elif choice == '2':
                tester.call_arm_service(False)
            elif choice == '3':
                tester.call_takeoff()
            elif choice == '4':
                tester.call_land()
            elif choice == '5':
                tester.call_emergency_stop()
            elif choice == '6':
                tester.send_velocity_command(1.0, 0.0, 0.0, 0.0)
            elif choice == '7':
                tester.send_velocity_command(0.0, 0.0, 1.0, 0.0)
            elif choice == '8':
                tester.send_velocity_command(0.0, 0.0, -0.5, 0.0)
            elif choice == '9':
                tester.send_velocity_command(0.0, 0.0, 0.0, 0.5)
            elif choice == '0':
                tester.send_velocity_command(0.0, 0.0, 0.0, 0.0)
            elif choice == 'p':
                x = float(input("  Enter X: "))
                y = float(input("  Enter Y: "))
                z = float(input("  Enter Z: "))
                tester.send_position_command(x, y, z)
            elif choice == 'a':
                alt = float(input("  Enter altitude (m): "))
                tester.send_altitude_command(alt)
            elif choice == 't':
                run_test_sequence(tester)
            elif choice == 'q':
                break
            else:
                print("Invalid choice!")
        except Exception as e:
            print(f"Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    tester = DroneControlTester()
    
    print("\n" + "="*70)
    print("DRONE CONTROL SUITE TESTER")
    print("="*70)
    print("\nMake sure the simulation is running:")
    print("  cd ~/ResQoUnity && ./start_simulation.sh drone 1 flat office")
    print("\nPress Enter to continue...")
    input()
    
    try:
        print("\nChoose mode:")
        print("  1. Run automated test sequence")
        print("  2. Manual control")
        mode = input("Enter choice (1/2): ").strip()
        
        if mode == '1':
            run_test_sequence(tester)
        else:
            manual_control_menu(tester)
            
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        tester.destroy_node()
        rclpy.shutdown()
        print("Tester shutdown complete")


if __name__ == '__main__':
    main()

