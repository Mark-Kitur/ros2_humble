import ollama, rclpy
from rclpy.node import Node
from std_msgs.msg import String
from  nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

tools =[
    {
        'type':'function',
        'function':{
            'name':'get_current_pose',
            'description':"Get the pose of the robot",
            'parameters':{
                'type':'object',
                'properties':{

                },
            },
        },
    },
    {
        'type':'function',
        'function':{
            'name':'move',
            'description':'Goto a direction',
            'parameters':{
                'type':'object',
                'properties':{
                    'direction':{
                        'type':'string',
                        'description':'motion direction',
                    },
                    'offset':{
                        'type':'string',
                        'description':'motion offset',
                    },
                },
            },
            'required':['direction','offset'],
        },
    },
    {
        'type':'function',
        'function':{
            'name':'generic_chat',
            'desacription':'chat',
            'parameters':{},
        },
    },
]
def string_to_float(value:str):
    cleande_value = "".join(c for c in value if c.isdigit() or c == '.' or c =='-')
    return float(cleande_value)

class OllamaNav(Node):
    def __init__(self):
        super().__init__("controll_robot")

        # publisher
        self.robot_res= self.create_publisher(String, 'response',10)
        self.robot_pub =self.create_publisher(String,'input_pos',10)
        self.timer_= self.create_timer(0.2,self.user_input)
        self.check_input =False

        # subscriber 
        self.robot_input =self.create_subscription(String, 'input_pos',self.requestion_cb,10)
        self.pose_sub = self.create_subscription(Odometry,'/odom',self.odom_cb,10)

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pos_x= 0.0
        self.pos_y =0.0

    def odom_cb(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
    
    def get_current_pose(self):
        return str(self.pos_x) + " "+ str(self.pos_y)
    
    def goal_response_callback(self, future):
        goal_handle =future.result()
        if not goal_handle.accepted:
            self.point_reached =True
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.point_reached =True

    def feedback_callabck(self, feedback):
        feedback=True
    
    def move(self, direction, offset):
        x,y =0.0, 0.0

        if direction == 'forward' or direction =='ahead':
            x= offset
        elif direction == 'backward' or direction == 'back':
            x=-offset
        elif direction == 'left':
            y =offset
        elif direction == 'right':
            y =-offset

        goal_msg= NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = self.pos_x+x
        goal_msg.pose.pose.position.y = self.pos_y+y
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y =0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w =1.0

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callabck)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def generic_chat(self):
        return "sorry, I cannot server or understand your request. Can you be more specific?"
    
    def requestion_cb(self, msg):
        self.get_logger().info(f"received request: {msg.data}")
        response=ollama.chat(model='llama3.2', messages=[{'role':'user','content': msg.data}], tools=tools)
        
        print('Repsonse: ', response)
        res= ''
        if 'message' in response and 'tool_calls' in response['message'] and len(response['message']['tool_calls'])>0:
            tools_calls = response['message']['tool_calls']
            tool_name = tools_calls[0]['function']['name']

            if tool_name == 'move':
                if 'arguments' in tools_calls[0]['function'] and len(tools_calls[0]['function']['arguments']):
                    arguments = tools_calls[0]['function']['arguments']
                    self.move(arguments['direction'], string_to_float(arguments['offset']))
                    res = 'Moving'

            elif tool_name == 'get_current_pose':
                pos = self.get_current_pose()
                res = 'I am at position ' + pos

            else:
                res = self.generic_chat()


        else:
            res = self.generic_chat()
        output = String()
        output.data =res
        self.robot_res.publish(output)
        self.check_input=False


    def user_input(self):
        if self.check_input ==False:
            mess= input('Enter message >>> ')
            msg= String()
            msg.data = mess
            self.check_input=True
            self.robot_pub.publish(msg=msg)
            self.get_logger().info(f"send: {mess}")

    

def main(args= None):
    rclpy.init(args=args)
    node = OllamaNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =="__main__":
    main()


