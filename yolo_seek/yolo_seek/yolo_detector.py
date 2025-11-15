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
        super().__init__("seekandgo_node")
        self.declare_parameter('kx',0.2)
        self.declare_parameter('kyaw',0.3)
        self.kx= self.get_parameter('kx').get_parameter_value().double_value
        self.kyaw= self.get_parameter('kyaw').get_parameter_value().double_value
        self.get_logger().info(f"kx:{self.kx}, kyaw:{self.kyaw}")

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel',10)
        self.obj_input_sub = self.create_subscription(String,'/object_to_seek',self.object_to_seek_input,1)
        self.yolo_input_sub = self.create_subscription(DetectedObjectList,'/yolo/detections/list',self.yolo_detections,10)
        self.subscription_= self.create_subscription(Image, '/camera_depth/image_raw',self.depth_image_callback,10)
        self.camera_info = self.create_subscription(CameraInfo, '/camera_depth/depth/camera_info', self.camera_info_callback,10)

        self.timer =self.create_timer(0.5, self.get_transform)
        self.object_to_seek= None
        self.new_object_to_seek = False
        self.obj_list = DetectedObjectList()
        self.bridge = CvBridge()
        self.depth_image = None
        self.tf_buffer =  Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        self.yaw=0

        self.first_tf_data = False
        self.first_cam_info = False
        self.first_depth = False
        self.first_yolo= False
        self.reaching_obj = False
        self.obj_reached =False

    def camera_info_callback(self,msg):
        camera_matrix = msg.k
        self.cx= camera_matrix[2]
        self.cy =camera_matrix[5]
        self.fx_inv = 1.0/ camera_matrix[0]
        self.fy_inv =1.0/camera_matrix[4]

        self.first_cam_info=True

    def depth_image_callback(self,msg):
        try:
            self.depth_image= self.bridge.imgmsg_to_cv2(msg,desired_encoding='32FC1')
            self.first_depth= True
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image ;{str(e)}")

    def yolo_detections(self,msg):
        self.obj_list =msg
        self.first_yolo= True

    def get_transform(self):
        try:
            transform =self.tf_buffer.lookup_transform('map','base_link', rclpy.time.Time(),Duration(seconds=0.1))
            rotation = transform.transform.rotation
            quaternion =[rotation.x, rotation.y, rotation.z, rotation.w]
            rotation =R.from_quat(quaternion)
            euler_angles = rotation.as_euler("xyz",degrees=False)
            self.yaw= euler_angles[2]
            self.first_tf_data=True
        except Exception as e:
            self.get_logger().warn(f"colud not get tansform :{e}")

    def retrivee_obj_from_list(self, obj):
        det_obj= DetectedObject()
        found=False
        i=0

        obj_list = self.obj_list
        while found ==False and i< len(obj_list.objects):
            if obj.lower() == obj_list.objects[i].class_name.lower():
                det_obj = obj_list.objects[i]
                found =True
            i +=1
        return found, det_obj
    
    def seek_object(self):
        self.obj_list.objects.clear()
        done= False
        found=False
        rate = self.create_rate(10)
        v= Twist()
        v.angular.z = 0.3
        total_yaw = 0
        prev_yaw= self.yaw
        laps=0
        while done == False and found == False:
            found ,_ = self.retrivee_obj_from_list(self.object_to_seek)
            self.cmd_vel_pub.publish(v)
            total_yaw =total_yaw +math.fabs(math.fabs(self.yaw)- math.fabs(prev_yaw))
            prev_yaw =self.yaw
            if total_yaw >6.2:
                laps+=1
                total_yaw=0.0

            if laps>1:
                done =True
            rate.sleep()
        v.angular.z= 0.0
        self.cmd_vel_pub.publish(v)
        return found
    def goto_object(self):
        rate= self.create_rate(10)
        reached=False
        done= False
        vel_cmd= Twist()

        while not done and not reached:
            found, det_obj =self.retrivee_obj_from_list(self.object_to_seek)

            if found:
                u= det_obj.canter_x
                v= det_obj.center_y
                c_z= self.depth_image[v,u]
                c_y = c_z *((v-self.cy)*self.fy_inv)
                c_x= c_z*((u-self.cx) *self.fx_inv)

                x= c_z
                y =- c_x

                e_x= math.fabs(x)
                e_y = math.fabs(y)

                vel_cmd.linear.x = self.kx*e_x

                if e_x<0.8:
                    vel_cmd.linear.x =0.0
                    if e_y <0.3:
                        self.obj_reached=True
                        self.get_logger().info("Object reached")
                        reached=True
                        done=True
                
                dir =1
                if y<0.0:
                    dir -=1
                vel_cmd.angular.z = dir *self.kyaw*e_y
                self.cmd_vel_pub.publish(vel_cmd)

                rate.sleep()
            else:
                self.get_logger().info("object lost")
                done= True
        vel_cmd.linear.x =0.0
        vel_cmd.angular.z=0.0
        self.cmd_vel_pub.publish(vel_cmd)
        return reached
    def main_loop(self):

        while not self.first_tf_data and not self.first_cam_info and not self.first_depth and not self.first_yolo:
            time.sleep(0.1)
            rate= self.create_rate(2)
            obj_found= False
            v= Twist()
            v.angular.z = 0.0
            self.cmd_vel_pub.publish(v)

            while rclpy.ok():
                if self.new_object_to_seekz:
                    self.reaching_obj=True
                    obj_found= self.seek_object()
                    if obj_found == True:
                        self.get_logger().info("requasted object seen in the scene , navigate to it")
                        reached = self.goto_object()
                        if reached :
                            self.new_object_to_seek=False

                        else:
                            print('Object not found... Try with another object')
                            self.reaching_obj=False

    def object_to_seek_input(self,msg):
        self.get_logger().info(f"new object to seek:{msg.data}")
        self.object_to_seek= msg.data
        self.new_object_to_seek=True
        self.obj_reached =False

    def run(self):
        main_loop_thread =Thread(target=self.main_loop, args=())
        main_loop_thread.start()
        rclpy.spin()

def main(args=None):
    rclpy.init(args=args)
    node=SeekAndGo()
    node.run()

if __name__ =="__main__":
    main()