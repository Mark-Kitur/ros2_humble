import rclpy
import time
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix

class BirdController(Node):
    def __init__(self):
        super().__init__('bird_controller')
        self.wing_pub = self.create_publisher(Float64MultiArray, '/wing_controller/commands', 10)
        self.tail_pub = self.create_publisher(Float64MultiArray, '/tail_controller/commands', 10)
        self.pos_subscriber = self.create_subscription(NavSatFix, '/ornithopter/gps/fix', self.pos_callback, 10)

        self.timer = self.create_timer(0.01, self.timer_callback) 
        self.start_time = self.get_clock().now()
        self.sample_rate =100

        # parameters
        self.flap_freq = 10      # Hz
        self.flap_amp = 1      # radians (reduced for stability)
        self.tail_amp = 0.52
        self.tail_freq = 1

        self.get_logger().info("BirdController started (position-mode)")

    def pos_callback(self, msg):
        self.get_logger().info(f"GPS lat={msg.latitude:.6f} logi={msg.longitude:.6f} alt={msg.altitude:.6f}")

    def timer_callback(self):
        t = (self.get_clock().now().nanoseconds - self.start_time.nanoseconds) / 1e9
        left_pos = self.flap_amp *np.sin(2*np.pi*self.flap_freq* t)
        right_pos = self.flap_amp *np.sin(2*np.pi*self.flap_freq*t +np.deg2rad(75))
        tail_angle = self.tail_amp * np.sin(2 * np.pi * self.tail_freq * t)

        # Publish torques
        wing_msg = Float64MultiArray()
        wing_msg.data = [left_pos, right_pos]
        self.wing_pub.publish(wing_msg)

        # Tail positions (same as before)
        tail_msg = Float64MultiArray()
        tail_msg.data = [tail_angle, tail_angle]
        self.tail_pub.publish(tail_msg)



def main(args=None):
    rclpy.init(args=args)
    node = BirdController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
