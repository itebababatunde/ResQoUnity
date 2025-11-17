#!/usr/bin/env python3
"""
Comprehensive Drone Diagnostics Test Suite
Provides detailed pass/fail status with actionable debugging data
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
from std_srvs.srv import Trigger, SetBool
import time
import math


class DroneDiagnostics(Node):
    def __init__(self):
        super().__init__('drone_diagnostics')
        
        # Subscribers for monitoring
        self.odom_sub = self.create_subscription(Odometry, '/robot0/odom', self.odom_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/robot0/cmd_vel', 10)
        self.cmd_pos_pub = self.create_publisher(PoseStamped, '/robot0/cmd_position', 10)
        self.cmd_alt_pub = self.create_publisher(Float32, '/robot0/cmd_altitude', 10)
        
        # Service clients
        self.arm_client = self.create_client(SetBool, '/robot0/arm')
        self.takeoff_client = self.create_client(Trigger, '/robot0/takeoff')
        self.land_client = self.create_client(Trigger, '/robot0/land')
        self.emergency_client = self.create_client(Trigger, '/robot0/emergency_stop')
        
        # State tracking
        self.current_position = None
        self.position_history = []
        
        self.get_logger().info('Drone Diagnostics initialized')
    
    def odom_callback(self, msg):
        """Track drone position"""
        self.current_position = msg.pose.pose.position
        self.position_history.append({
            'time': time.time(),
            'x': self.current_position.x,
            'y': self.current_position.y,
            'z': self.current_position.z
        })
        # Keep only last 100 positions
        if len(self.position_history) > 100:
            self.position_history.pop(0)
    
    def wait_for_position(self, timeout=5.0):
        """Wait for first position reading"""
        start = time.time()
        while self.current_position is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.current_position is not None
    
    def get_position(self):
        """Get current position, spinning to get fresh data"""
        # Always spin a few times to ensure we get the latest odometry
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.current_position
    
    def call_service_sync(self, client, request, timeout=5.0):
        """Synchronous service call with timeout"""
        if not client.wait_for_service(timeout_sec=2.0):
            return None, "Service not available"
        
        future = client.call_async(request)
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not future.done():
            return None, "Service call timed out"
        
        try:
            response = future.result()
            return response, None
        except Exception as e:
            return None, str(e)


def print_header(text):
    """Print formatted test header"""
    print(f"\n{'='*80}")
    print(f"  {text}")
    print(f"{'='*80}")


def print_test(number, name):
    """Print test name"""
    print(f"\n[TEST {number}] {name}")
    print("-" * 80)


def print_pass(message, details=""):
    """Print pass message"""
    print(f"✅ PASS: {message}")
    if details:
        print(f"   {details}")


def print_fail(message, details=""):
    """Print fail message"""
    print(f"❌ FAIL: {message}")
    if details:
        print(f"   {details}")


def print_info(message):
    """Print info message"""
    print(f"ℹ️  INFO: {message}")


def run_diagnostics(tester):
    """Run comprehensive diagnostic tests"""
    results = {
        'tests': [],
        'passed': 0,
        'failed': 0
    }
    
    print_header("DRONE CONTROL DIAGNOSTICS")
    print("This test suite provides detailed pass/fail status for debugging")
    
    time.sleep(1)
    
    # ========================================================================
    # TEST 1: Odometry Check
    # ========================================================================
    print_test(1, "Odometry Publishing")
    
    if not tester.wait_for_position(timeout=5.0):
        print_fail("No odometry data received within 5 seconds")
        print_info("Check: Is simulation running? Is ROS2 bridge active?")
        results['tests'].append({'name': 'Odometry', 'status': 'FAIL'})
        results['failed'] += 1
        return results
    
    pos = tester.get_position()
    print_pass("Odometry publishing", f"Position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
    initial_z = pos.z
    print_info(f"Initial altitude: {initial_z:.3f}m")
    results['tests'].append({'name': 'Odometry', 'status': 'PASS'})
    results['passed'] += 1
    
    # ========================================================================
    # TEST 2: Arm Service
    # ========================================================================
    print_test(2, "Arm Service")
    
    request = SetBool.Request()
    request.data = True
    response, error = tester.call_service_sync(tester.arm_client, request)
    
    if error or not response:
        print_fail(f"Arm service failed: {error}")
        print_info("Check: Is /drone/arm service available? Run: ros2 service list | grep arm")
        results['tests'].append({'name': 'Arm Service', 'status': 'FAIL'})
        results['failed'] += 1
        return results
    
    if response.success:
        print_pass("Drone armed successfully", response.message)
        results['tests'].append({'name': 'Arm Service', 'status': 'PASS'})
        results['passed'] += 1
    else:
        print_fail("Arm service returned failure", response.message)
        results['tests'].append({'name': 'Arm Service', 'status': 'FAIL'})
        results['failed'] += 1
        return results
    
    time.sleep(1)
    
    # ========================================================================
    # TEST 3: Altitude Hold (Armed, No Commands)
    # ========================================================================
    print_test(3, "Altitude Hold After Arming")
    
    pos_before = tester.get_position()
    print_info(f"Position before: Z={pos_before.z:.3f}m")
    
    print_info("Waiting 5 seconds to check altitude stability...")
    time.sleep(5)
    
    pos_after = tester.get_position()
    altitude_change = abs(pos_after.z - pos_before.z)
    print_info(f"Position after:  Z={pos_after.z:.3f}m")
    print_info(f"Altitude change: {altitude_change:.3f}m")
    
    if altitude_change < 0.5:
        print_pass(f"Altitude stable (drift: {altitude_change:.3f}m)")
        results['tests'].append({'name': 'Altitude Hold', 'status': 'PASS'})
        results['passed'] += 1
    else:
        print_fail(f"Altitude unstable (drift: {altitude_change:.3f}m > 0.5m threshold)")
        print_info("Check: Is drone falling? Is gravity compensation working?")
        results['tests'].append({'name': 'Altitude Hold', 'status': 'FAIL'})
        results['failed'] += 1
    
    # ========================================================================
    # TEST 4: Takeoff Service
    # ========================================================================
    print_test(4, "Takeoff Service")
    
    request = Trigger.Request()
    response, error = tester.call_service_sync(tester.takeoff_client, request)
    
    if error or not response:
        print_fail(f"Takeoff service failed: {error}")
        print_info("Check: Is /drone/takeoff service available?")
        results['tests'].append({'name': 'Takeoff Service', 'status': 'FAIL'})
        results['failed'] += 1
    elif response.success:
        print_pass("Takeoff command accepted", response.message)
        results['tests'].append({'name': 'Takeoff Service', 'status': 'PASS'})
        results['passed'] += 1
    else:
        print_fail("Takeoff service returned failure", response.message)
        results['tests'].append({'name': 'Takeoff Service', 'status': 'FAIL'})
        results['failed'] += 1
    
    # ========================================================================
    # TEST 5: Altitude Change After Takeoff
    # ========================================================================
    print_test(5, "Altitude Change After Takeoff")
    
    pos_before_takeoff = tester.get_position()
    print_info(f"Altitude before takeoff: {pos_before_takeoff.z:.3f}m")
    print_info("Waiting 10 seconds for takeoff...")
    
    time.sleep(10)
    
    pos_after_takeoff = tester.get_position()
    altitude_gain = pos_after_takeoff.z - pos_before_takeoff.z
    print_info(f"Altitude after takeoff: {pos_after_takeoff.z:.3f}m")
    print_info(f"Altitude gain: {altitude_gain:+.3f}m")
    
    if altitude_gain > 0.2:
        print_pass(f"Drone climbed {altitude_gain:.3f}m")
        results['tests'].append({'name': 'Takeoff Altitude Gain', 'status': 'PASS'})
        results['passed'] += 1
    elif abs(altitude_gain) < 0.05:
        print_fail(f"No altitude change detected ({altitude_gain:.3f}m)")
        print_info("Physics reports position change but actual position unchanged")
        print_info("Possible: Velocities not being applied to physics state")
        results['tests'].append({'name': 'Takeoff Altitude Gain', 'status': 'FAIL'})
        results['failed'] += 1
    else:
        print_fail(f"Drone descended {altitude_gain:.3f}m (should climb)")
        print_info("Check: Is vertical control inverted? Are forces being applied?")
        results['tests'].append({'name': 'Takeoff Altitude Gain', 'status': 'FAIL'})
        results['failed'] += 1
    
    # ========================================================================
    # TEST 6: Position Command
    # ========================================================================
    print_test(6, "Position Control (Horizontal Movement)")
    
    pos_before_move = tester.get_position()
    print_info(f"Position before: ({pos_before_move.x:.3f}, {pos_before_move.y:.3f}, {pos_before_move.z:.3f})")
    
    # Send position command
    cmd = PoseStamped()
    cmd.header.frame_id = 'odom'
    cmd.header.stamp = tester.get_clock().now().to_msg()
    target_x, target_y, target_z = 1.0, 0.5, pos_before_move.z + 0.5
    cmd.pose.position.x = target_x
    cmd.pose.position.y = target_y
    cmd.pose.position.z = target_z
    
    print_info(f"Commanding position: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})")
    tester.cmd_pos_pub.publish(cmd)
    
    print_info("Waiting 15 seconds to reach target...")
    time.sleep(15)
    
    pos_after_move = tester.get_position()
    print_info(f"Position after: ({pos_after_move.x:.3f}, {pos_after_move.y:.3f}, {pos_after_move.z:.3f})")
    
    error_x = abs(pos_after_move.x - target_x)
    error_y = abs(pos_after_move.y - target_y)
    error_z = abs(pos_after_move.z - target_z)
    total_error = math.sqrt(error_x**2 + error_y**2 + error_z**2)
    
    print_info(f"Position error: ({error_x:.3f}, {error_y:.3f}, {error_z:.3f}) = {total_error:.3f}m")
    
    if total_error < 0.5:
        print_pass(f"Reached target position (error: {total_error:.3f}m)")
        results['tests'].append({'name': 'Position Control', 'status': 'PASS'})
        results['passed'] += 1
    else:
        print_fail(f"Did not reach target (error: {total_error:.3f}m > 0.5m threshold)")
        
        # Diagnose why
        horizontal_movement = math.sqrt((pos_after_move.x - pos_before_move.x)**2 + 
                                       (pos_after_move.y - pos_before_move.y)**2)
        vertical_movement = abs(pos_after_move.z - pos_before_move.z)
        
        if horizontal_movement < 0.1 and vertical_movement < 0.1:
            print_info("No movement detected - drone is stuck")
            print_info("Check: Are velocities being applied? Is physics running?")
        elif horizontal_movement < 0.1:
            print_info(f"No horizontal movement (only vertical: {vertical_movement:.3f}m)")
            print_info("Check: Is XY position control working?")
        else:
            print_info(f"Drone moved {horizontal_movement:.3f}m horizontally but not to target")
            print_info("Check: PID gains, maximum velocity limits")
        
        results['tests'].append({'name': 'Position Control', 'status': 'FAIL'})
        results['failed'] += 1
    
    # ========================================================================
    # TEST 7: Land Service
    # ========================================================================
    print_test(7, "Land Service")
    
    request = Trigger.Request()
    response, error = tester.call_service_sync(tester.land_client, request, timeout=10.0)
    
    if error:
        print_fail(f"Land service failed: {error}")
        print_info("Common causes: Service timeout, callback exception, ROS2 node not spinning")
        results['tests'].append({'name': 'Land Service', 'status': 'FAIL'})
        results['failed'] += 1
    elif response and response.success:
        print_pass("Land command accepted", response.message)
        results['tests'].append({'name': 'Land Service', 'status': 'PASS'})
        results['passed'] += 1
    else:
        print_fail("Land service returned failure", response.message if response else "No response")
        results['tests'].append({'name': 'Land Service', 'status': 'FAIL'})
        results['failed'] += 1
    
    return results


def main(args=None):
    rclpy.init(args=args)
    
    tester = DroneDiagnostics()
    
    print("\n" + "="*80)
    print("DRONE CONTROL DIAGNOSTICS TEST SUITE")
    print("="*80)
    print("\nEnsure simulation is running:")
    print("  ./start_simulation.sh go2 1 flat office")
    print("\nPress Enter to begin...")
    input()
    
    try:
        results = run_diagnostics(tester)
        
        # Print summary
        print_header("TEST SUMMARY")
        print(f"\nTotal Tests: {results['passed'] + results['failed']}")
        print(f"✅ Passed: {results['passed']}")
        print(f"❌ Failed: {results['failed']}")
        
        print("\nDetailed Results:")
        for test in results['tests']:
            status_symbol = "✅" if test['status'] == 'PASS' else "❌"
            print(f"  {status_symbol} {test['name']}: {test['status']}")
        
        if results['failed'] == 0:
            print("\n🎉 ALL TESTS PASSED! Drone control is fully functional.")
        else:
            print(f"\n⚠️  {results['failed']} test(s) failed. Review output above for debugging info.")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    finally:
        tester.destroy_node()
        rclpy.shutdown()
        print("\nDiagnostics complete")


if __name__ == '__main__':
    main()

