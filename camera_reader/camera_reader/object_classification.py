import rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
from std_msgs.msg import String


class YoloNode(Node):
    def __init__(self):
        super().__init__("yolo_node")
        self.subscription_= self.create_subscription(
            Image, '/camera/image_raw', self.image_callback,10
        )
        self.detections_publisher= self.create_publisher(
            String,'/yolo/detections',10
        )
        self.bridge =CvBridge()
        self.model= YOLO('yolov8n.pt')

    def image_callback(self,msg):
        frame= self.bridge.imgmsg_to_cv2(msg,'bgr8')
        results= self.model(frame)

        self.display_results(frame,results)
        detections =self.format_detections(results)
        detection_msg= String()
        detection_msg.data =detections
        self.detections_publisher.publish(detection_msg)


    def display_results(self,frame,results):
        for result in results:
            boxes= results.boxes
            for box in boxes:
                x1,y1,x2,y2 = map(int, box.xyxy[0])
                class_id = int(box.cls[0])
                confidence= box.conf[0]
                label = self.model.names[class_id]

                cv2.rectangle(frame,(x1,y1),(x2,y2),(0, 255, 0),2)
                cv2.putText(frame,f"{label} {confidence:.2f}", (x1,y1-10), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0, 255,0),2)
        cv2.imshow('YOLOv* Detections', frame)
        cv2.waitKey(1)

    def format_detections(self,results):
        detections =[]
        for result in results:
            boxes = result.boxes
            for box in boxes:
                class_id = int(box.cls[0])
                label = self.model.names[class_id]
                detections.append(label)

        return ', '.join(detections) if detections else "No objects detected"
    
def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down YOLO node")
    
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ =="__main__":
    main()