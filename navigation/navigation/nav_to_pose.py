import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from threading import Thread
import time
import math

def get_cords():
    x,y,yaw = map(float,input("Enter x,y,yaw (seperated by spces): ").split())
    return x,y,yaw


class NavigateToPoseClient(Node):
    def __init__(self):
        super().__init__("nagigate_to_pose")
        self._action_client=ActionClient(self,NavigateToPose,'navigate_to_pose')

    def send_goal(self,x,y,yaw):
        goal_msg= NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id= 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y =y

        goal_msg.pose.pose.orientation.x= 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw/2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw/2)

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal=goal_msg, feedback_callback=self.feeback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self,future):
        goal_handle= future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal accepted')
            return
        self.get_logger().info("goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self,feedback):
        feedback= feedback.feedback
        self.get_logger().info(f'received feeback: {feedback.feedback}')

    def get_result_callback(self,future):
        result = future.result().future
        self.get_logger().info(f"Goal completed with status: {future.result().status}")
        self.get_logger().info(f"Final pose: x={result.pose.pose.position.x:.2f}, "
                               f"y={result.pose.pose.position.y:.2f}")

    def spin_thread(self):
        rclpy.spin(self)

    def run (self):
        spin_thread = Thread(target=self.spin_thread,args=())
        spin_thread.start()
        while True:
            self.get_logger().info("Please enter coordinates")
            x,y,yaw= get_cords() 
            self.send_goal(x,y,yaw)


def main(args=None):
    rclpy.init(args=args)
    node = NavigateToPoseClient()
    node.run()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ =="__main__":
    main()    