'''
updated to close loop all in the ollama_store.py
'''

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String


# class UserInput(Node):
#     def __init__(self):
#         super().__init__("read_user")


#         self.pub_user= self.create_publisher(String,'input_request',10)
#         self.check_input=False
#         self.get_logger().info("starting user server...")

#         self.timer = self.create_timer(0.1,self.user_input)

#     def user_input(self):
#             try:
#                 message = input("Enter message >>> ")
#                 msg=String()
#                 msg.data =message
                
#                 self.pub_user.publish(msg)
#                 self.get_logger().info("Message received")

#             except Exception as e:
#                 self.get_logger().info("Error! no input received")


# def main(args=None):
#     rclpy.init(args=args)
#     node =UserInput()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__=="__main__":
#     main()