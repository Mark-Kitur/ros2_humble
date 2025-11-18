import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
import threading
import math

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))

class SeekObject(Node):
    def __init__(self):
        super().__init__('seek_object_node')

        # Publisher to control robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Goal coordinates
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_set = False

        # Control gains
        self.k_linear = 0.5
        self.k_angular = 1.5
        self.distance_tolerance = 0.2

        # Start timer to move robot
        self.timer = self.create_timer(1, self.move_toward_goal)

        # Start user input thread
        threading.Thread(target=self.get_user_goal, daemon=True).start()

    def get_user_goal(self):
        """Ask the user for goal coordinates without blocking ROS"""
        while rclpy.ok():
            try:
                x = float(input("Enter goal X position: "))
                y = float(input("Enter goal Y position: "))
                self.goal_x = x
                self.goal_y = y
                self.goal_set = True
                self.get_logger().info(f"New goal set: ({x}, {y})")
            except Exception as e:
                self.get_logger().error(f"Invalid input: {e}")

    def move_toward_goal(self):
        """Move robot toward the goal using TF"""
        if not self.goal_set:
            return

        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            self.get_logger().info(f"x:{x:.2f} y:{y:.2f}\n")

            q = trans.transform.rotation
            yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

            # Compute distance and angle to goal
            dx = self.goal_x - x
            dy = self.goal_y - y
            distance = math.sqrt(dx**2 + dy**2)
            goal_yaw = math.atan2(dy, dx)
            yaw_error = normalize_angle(goal_yaw - yaw)

            cmd = Twist()
            # move in x direction
            if dx > self.distance_tolerance:
                cmd.linear.x = dx   # max speed limit
                # cmd.angular.z = self.k_angular * yaw_error
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.goal_set = False
                self.get_logger().info("Goal reached!")

            self.cmd_pub.publish(cmd)

        except Exception as e:
            self.get_logger().warn(f"TF not ready: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SeekObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
