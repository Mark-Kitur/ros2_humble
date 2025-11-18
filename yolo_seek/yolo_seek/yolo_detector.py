from yolo_msgs.msg import DetectedObject, DetectedObjectList
import rclpy, math, time
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from threading import Thread


class SeekAndGo(Node):
    def __init__(self):
        super().__init__("seek_and_go_node")
        self.declare_parameter('kx', 0.2)
        self.declare_parameter('kyaw', 0.3)
        self.kx = self.get_parameter('kx').get_parameter_value().double_value
        self.kyaw = self.get_parameter('kyaw').get_parameter_value().double_value
        self.get_logger().info(f"kx:{self.kx}, kyaw:{self.kyaw}")

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.obj_input_sub = self.create_subscription(String, '/object_to_seek', self.object_to_seek_input, 1)
        self.yolo_input_sub = self.create_subscription(DetectedObjectList, '/yolo/detections/list', self.yolo_detections, 10)
        self.depth_sub = self.create_subscription(Image, '/camera_depth/depth/image_raw', self.depth_image_callback, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, '/camera_depth/depth/camera_info', self.camera_info_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State variables
        self.object_to_seek = None
        self.new_object_to_seek = False
        self.obj_list = DetectedObjectList()
        self.bridge = CvBridge()
        self.depth_image = None
        self.yaw = 0.0

        self.first_tf_data = False
        self.first_cam_info = False
        self.first_depth = False
        self.first_yolo = False

        # Timer to update TF
        self.create_timer(0.1, self.get_transform)

    # ------------------ Callbacks ------------------
    def camera_info_callback(self, msg):
        k = msg.k
        self.cx = k[2]
        self.cy = k[5]
        self.fx_inv = 1.0 / k[0]
        self.fy_inv = 1.0 / k[4]
        self.first_cam_info = True

    def depth_image_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self.first_depth = True
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {str(e)}")

    def yolo_detections(self, msg):
        self.obj_list = msg
        self.first_yolo = True

    def get_transform(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), Duration(seconds=0.1))
            q = tf.transform.rotation
            r = R.from_quat([q.x, q.y, q.z, q.w])
            self.yaw = r.as_euler("xyz", degrees=False)[2]
            self.first_tf_data = True
        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")

    # ------------------ Object handling ------------------
    def retrivee_obj_from_list(self, obj_name):
        for o in self.obj_list.objects:
            if obj_name.lower() == o.class_name.lower():
                return True, o
        return False, DetectedObject()

    # ------------------ Behaviors ------------------
    def seek_object(self):
        self.obj_list.objects.clear()
        rate = self.create_rate(10)
        v = Twist()
        v.angular.z = 0.3  # Rotate continuously
        total_yaw = 0.0
        prev_yaw = self.yaw
        laps = 0
        found = False

        while not found and rclpy.ok():
            found, _ = self.retrivee_obj_from_list(self.object_to_seek)
            self.cmd_vel_pub.publish(v)
            total_yaw += abs(abs(self.yaw) - abs(prev_yaw))
            prev_yaw = self.yaw
            if total_yaw > 6.2:  # roughly one full rotation
                laps += 1
                total_yaw = 0.0
            if laps > 2:  # give up after 2 rotations
                break
            rate.sleep()

        v.angular.z = 0.0
        self.cmd_vel_pub.publish(v)
        return found

    def goto_object(self):
        rate = self.create_rate(10)
        vel_cmd = Twist()
        reached = False

        while not reached and rclpy.ok():
            found, obj = self.retrivee_obj_from_list(self.object_to_seek)
            if not found:
                self.get_logger().info("Object lost")
                break

            u = obj.canter_x
            v = obj.center_y
            z = self.depth_image[v, u]
            x = -z * ((u - self.cx) * self.fx_inv)
            y = z * ((v - self.cy) * self.fy_inv)

            # Control
            vel_cmd.linear.x = self.kx * abs(z)
            if abs(z) < 0.8:
                vel_cmd.linear.x = 0.0
                if abs(x) < 0.3:
                    reached = True
                    self.get_logger().info("Object reached")

            vel_cmd.angular.z = self.kyaw * x * (-1 if x < 0 else 1)
            self.cmd_vel_pub.publish(vel_cmd)
            rate.sleep()

        vel_cmd.linear.x = 0.0
        vel_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(vel_cmd)
        return reached

    # ------------------ Main Loop ------------------
    def main_loop(self):
        while not (self.first_tf_data and self.first_cam_info and self.first_depth and self.first_yolo):
            time.sleep(0.1)

        while rclpy.ok():
            if self.new_object_to_seek:
                self.reaching_obj = True
                found = self.seek_object()
                if found:
                    self.get_logger().info("Requested object detected, moving towards it")
                    self.goto_object()
                    self.new_object_to_seek = False
                else:
                    self.get_logger().info("Object not found after full rotation")
                    self.reaching_obj = False
            time.sleep(0.1)

    # ------------------ Object input ------------------
    def object_to_seek_input(self, msg):
        self.get_logger().info(f"New object to seek: {msg.data}")
        self.object_to_seek = msg.data
        self.new_object_to_seek = True
        self.obj_reached = False
        # Immediately start rotation
        t = Twist()
        t.angular.z = 0.3
        self.cmd_vel_pub.publish(t)

    # ------------------ Run ------------------
    def run(self):
        Thread(target=self.main_loop).start()
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    node = SeekAndGo()
    node.run()


if __name__ == "__main__":
    main()
