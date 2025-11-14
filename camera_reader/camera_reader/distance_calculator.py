import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2


class DepthDistanceCalculator(Node):
    def __init__(self):
        super().__init__("depth_distance_calc")
        self.subscription_image = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw', self.image_callback,10
        )
        self.subscription_depth = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw', self.depth_callback,10
        )
        self.bridge = CvBridge()

        self.color_image = None
        self.depth_image = None
        self.dist_coeffs =None
        self.distance =None

        self.publisher_= self.create_publisher(Image, "/annoted_image", 10)


    def image_callback(self,msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        if self.color_image is not None and self.distance is not None:
            self.display_annotated_image()

    def depth_callback(self,msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        if self.depth_image is not None:
            self.calculated_distance()

    def calculated_distance(self):
        if self.depth_image is None:
            self.get_logger().warning("Depth image is not available")
            return
        
        height, width =self.depth_image.shape

        center_x= width //2
        center_y= height //2

        self.distance =self.depth_image[center_y, center_x] *0.001

        if not np.isfinite(self.distance):
            self.get_logger().warning("Invalid depth value at center point")
            self.distance=None

    def display_annotated_image(self):
        annoted_image = self.color_image.copy()
        height, width ,_ =annoted_image.shape

        center_x = width //2
        center_y = height //2

        cv2.circle(annoted_image,(center_x,center_y),5 , (0,0,255),-1)

        if self.distance is  not None and np.isfinite(self.distance):
            distance_text = f"{self.distance:.2f}"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            color = (255,255,255)
            thickness =2
            text_pos= (center_x-50, center_y-10)
            cv2.putText(annoted_image,distance_text,text_pos,font, font_scale,color,thickness)

        imag_msg= self.bridge.cv2_to_imgmsg(annoted_image,encoding='bgr8')
        self.publisher_(imag_msg)

def main(args=None):
    rclpy.init(args=args)
    node= DepthDistanceCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =="__main__":
    main()