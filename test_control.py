#!/usr/bin/env python3
"""
Test script to control Go2 robots and drone via ROS2 topics.

Usage:
    Terminal 1: cd ~/ResQoUnity && ./start_simulation.sh
    Terminal 2: python3 test_control.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class AgentController(Node):
    def __init__(self):
        super().__init__('agent_controller')
        
        # Create publishers for robots
        self.robot0_pub = self.create_publisher(Twist, 'robot0/cmd_vel', 10)
        self.robot1_pub = self.create_publisher(Twist, 'robot1/cmd_vel', 10)
        
        # Create publisher for drone
        self.drone_pub = self.create_publisher(Twist, 'drone/cmd_vel', 10)
        
        self.get_logger().info('Agent Controller initialized')
        self.get_logger().info('Publishers created for:')
        self.get_logger().info('  - robot0/cmd_vel')
        self.get_logger().info('  - robot1/cmd_vel')
        self.get_logger().info('  - drone/cmd_vel')
    
    def move_robot(self, robot_id, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """Send velocity command to robot"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        
        if robot_id == 0:
            self.robot0_pub.publish(msg)
            self.get_logger().info(f'Robot 0: linear=({linear_x:.2f}, {linear_y:.2f}), angular={angular_z:.2f}')
        elif robot_id == 1:
            self.robot1_pub.publish(msg)
            self.get_logger().info(f'Robot 1: linear=({linear_x:.2f}, {linear_y:.2f}), angular={angular_z:.2f}')
    
    def move_drone(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_z=0.0):
        """Send velocity command to drone"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = linear_z
        msg.angular.z = angular_z
        
        self.drone_pub.publish(msg)
        self.get_logger().info(f'Drone: linear=({linear_x:.2f}, {linear_y:.2f}, {linear_z:.2f}), angular={angular_z:.2f}')
    
    def stop_all(self):
        """Stop all agents"""
        self.move_robot(0, 0.0, 0.0, 0.0)
        self.move_robot(1, 0.0, 0.0, 0.0)
        self.move_drone(0.0, 0.0, 0.0, 0.0)
        self.get_logger().info('All agents stopped')


def demo_sequence(controller):
    """Run a demo sequence"""
    print("\n" + "="*60)
    print("DEMO SEQUENCE: Testing Robot and Drone Control")
    print("="*60)
    
    # Test 1: Move Robot 0 forward
    print("\n[Test 1] Moving Robot 0 forward for 3 seconds...")
    controller.move_robot(0, linear_x=0.3)
    time.sleep(3)
    controller.stop_all()
    time.sleep(1)
    
    # Test 2: Move Robot 1 forward
    print("\n[Test 2] Moving Robot 1 forward for 3 seconds...")
    controller.move_robot(1, linear_x=0.3)
    time.sleep(3)
    controller.stop_all()
    time.sleep(1)
    
    # Test 3: Turn Robot 0
    print("\n[Test 3] Turning Robot 0 for 2 seconds...")
    controller.move_robot(0, angular_z=0.5)
    time.sleep(2)
    controller.stop_all()
    time.sleep(1)
    
    # Test 4: Move Drone up
    print("\n[Test 4] Moving Drone UP for 3 seconds...")
    controller.move_drone(linear_z=1.0)
    time.sleep(3)
    controller.stop_all()
    time.sleep(1)
    
    # Test 5: Move Drone forward
    print("\n[Test 5] Moving Drone FORWARD for 3 seconds...")
    controller.move_drone(linear_x=1.0)
    time.sleep(3)
    controller.stop_all()
    time.sleep(1)
    
    # Test 6: Move Drone down
    print("\n[Test 6] Moving Drone DOWN for 3 seconds...")
    controller.move_drone(linear_z=-0.5)
    time.sleep(3)
    controller.stop_all()
    
    print("\n" + "="*60)
    print("DEMO COMPLETE!")
    print("="*60)


def manual_control_menu(controller):
    """Interactive menu for manual control"""
    while True:
        print("\n" + "="*60)
        print("MANUAL CONTROL MENU")
        print("="*60)
        print("Robot Controls:")
        print("  1. Move Robot 0 forward")
        print("  2. Move Robot 1 forward")
        print("  3. Turn Robot 0 left")
        print("  4. Turn Robot 1 left")
        print("  5. Stop all robots")
        print("\nDrone Controls:")
        print("  6. Move Drone UP")
        print("  7. Move Drone DOWN")
        print("  8. Move Drone FORWARD")
        print("  9. Move Drone BACKWARD")
        print("  0. Stop drone")
        print("\nOther:")
        print("  d. Run demo sequence")
        print("  q. Quit")
        print("="*60)
        
        choice = input("Enter choice: ").strip().lower()
        
        if choice == '1':
            controller.move_robot(0, linear_x=0.5)
        elif choice == '2':
            controller.move_robot(1, linear_x=0.5)
        elif choice == '3':
            controller.move_robot(0, angular_z=0.5)
        elif choice == '4':
            controller.move_robot(1, angular_z=0.5)
        elif choice == '5':
            controller.stop_all()
        elif choice == '6':
            controller.move_drone(linear_z=1.0)
        elif choice == '7':
            controller.move_drone(linear_z=-1.0)
        elif choice == '8':
            controller.move_drone(linear_x=1.0)
        elif choice == '9':
            controller.move_drone(linear_x=-1.0)
        elif choice == '0':
            controller.move_drone(0.0, 0.0, 0.0)
        elif choice == 'd':
            demo_sequence(controller)
        elif choice == 'q':
            controller.stop_all()
            break
        else:
            print("Invalid choice!")


def main(args=None):
    rclpy.init(args=args)
    
    controller = AgentController()
    
    print("\n" + "="*60)
    print("AGENT CONTROLLER STARTED")
    print("="*60)
    print("\nMake sure the simulation is running!")
    print("Start with: cd ~/ResQoUnity && ./start_simulation.sh")
    print("\nPress Enter to continue...")
    input()
    
    try:
        # Run demo or manual control
        print("\nChoose mode:")
        print("  1. Run demo sequence")
        print("  2. Manual control")
        mode = input("Enter choice (1/2): ").strip()
        
        if mode == '1':
            demo_sequence(controller)
        else:
            manual_control_menu(controller)
            
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        controller.stop_all()
        controller.destroy_node()
        rclpy.shutdown()
        print("Controller shutdown complete")


if __name__ == '__main__':
    main()

