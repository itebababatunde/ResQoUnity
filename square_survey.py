#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool, Trigger
import time

class SquareSurvey(Node):
    def __init__(self):
        super().__init__('square_survey')
        self.pos_pub = self.create_publisher(PoseStamped, '/drone/cmd_position', 10)
        self.arm_client = self.create_client(SetBool, '/drone/arm')
        self.takeoff_client = self.create_client(Trigger, '/drone/takeoff')
        self.land_client = self.create_client(Trigger, '/drone/land')
    
    def goto(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.pos_pub.publish(msg)

def main():
    rclpy.init()
    drone = SquareSurvey()
    
    # Arm (drone already hovering at 2.5m, skip takeoff)
    print("Arming drone...")
    req = SetBool.Request()
    req.data = True
    drone.arm_client.call(req)
    time.sleep(2)
    
    # Survey 5x5 meter square (drone already at ~2.5m)
    print("Starting square survey...")
    corners = [(0,0,2.5), (5,0,2.5), (5,5,2.5), (0,5,2.5), (0,0,2.5)]
    
    for x, y, z in corners:
        print(f"Going to ({x}, {y}, {z})")
        drone.goto(x, y, z)
        time.sleep(8)  # Wait to reach waypoint
    
    # Land
    print("Landing...")
    drone.land_client.call(Trigger.Request())
    time.sleep(5)
    
    print("Mission complete!")
    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

