#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose

class OrnithopterTracker(Node):
    def __init__(self):
        super().__init__('ornithopter_tracker')
        
        # GPS subscriber
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/ornithopter/gps/fix',
            self.gps_callback,
            10)
            
        # Ground Truth subscriber
        self.gt_sub = self.create_subscription(
            Pose,
            '/ornithopter/ground_truth/pose',
            self.gt_callback,
            10)
        
        self.get_logger().info("🛰️ Ornithopter Tracker Started")
        self.get_logger().info("📍 GPS Topic: /ornithopter/gps/fix")
        self.get_logger().info("🎯 Ground Truth Topic: /ornithopter/ground_truth/pose")
        
    def gps_callback(self, msg):
        # GPS coordinates (latitude = X, longitude = Y, altitude = Z)
        x = msg.latitude
        y = msg.longitude  
        z = msg.altitude
        
        print(f"🛰️  GPS -> X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
        
    def gt_callback(self, msg):
        # Direct XYZ coordinates from Gazebo
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        
        print(f"🎯 GROUND TRUTH -> X={x:.2f}, Y={y:.2f}, Z={z:.2f}")

def main():
    rclpy.init()
    node = OrnithopterTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nTracker stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()