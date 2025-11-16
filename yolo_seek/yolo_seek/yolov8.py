import rclpy, cv2
from yolo_msgs.msg import DetectedObject,DetectedObjectList
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloNode(Node):
    def __init__(self):
        super().__init__("yolo_node")
        self.declare_parameter('camera_topic','/camera_depth/image_raw')
        camera_topic_param = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.subscription = self.create_subscription(Image, camera_topic_param, self.image_callback,10)
        self.detection_publisher = self.create_publisher(DetectedObjectList,'/yolo/detections/list',10)
        self.det_image_publisher = self.create_publisher(Image, '/yolo/detections/debug',10)

        self.bridge= CvBridge()
        self.model = YOLO("yolov8n.pt")

        self.obj_list = DetectedObjectList()
        self.get_logger().info('Yolo Node has been started')

    def image_callback(self,msg):
        frame =self.bridge.imgmsg_to_cv2(msg,'bgr8')
        results = self.model(frame)
        self.display_results(frame, results)

    def display_results(self,frame, results):
        # draw bounding boxes

        self.obj_list=DetectedObjectList()

        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1,y1,x2,y2 = map(int, box.xyxy[0])
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                label = self.model.names[class_id]
                cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255 ,0),2)
                cv2.circle(frame,(int((x2+x1)/2),int((y2+y1)/2)),5,(0,0,255),-1)
                cv2.putText(frame, f'{label} {confidence:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                obj =DetectedObject()
                obj.class_name=label
                obj.confidence= confidence

                obj.canter_x= int((x2+x1)/2)
                obj.center_y=int((y1+y2)/2)
                self.obj_list.objects.append(obj)
        self.detection_publisher.publish(self.obj_list)
        imag_msg= self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.det_image_publisher.publish(imag_msg)

    def format_detections(self,results):
        detections =[]

        for result in results:
            boxes = result.boxes
            for box in boxes:
                class_id = int(box.cls[0])
                label = self.model.names[class_id]
                detections.append(label)

        return ', '.join(detections) if detections else "no objects detected"
    
def main(args=None):
    rclpy.init(args=args)
    node= YoloNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down YOLO")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__=="__main__":
    main()    


