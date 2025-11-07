import gymnasium as gym
from gymnasium import spaces, Env
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool
import rclpy, threading, time, math 


from threading import Thread
import numpy as np

def bound_angle(angle):
    bounded_angle = (angle + math.pi) %(2*math.pi) - math.pi
    return bounded_angle

class CarPoleROS2Env(Env):
    def __init__(self):
        super().__init__()

        self.action_space = spaces.Box(low=-15, high=15, shape=(1,), dtype=np.float32)
        high = np.array([5.0, 1.0, math.pi,8.5],np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)
        rclpy.init(args=None)
        self.node= rclpy.create_node("cartpole_gym_env")
        self.cartpole_eff_pub = self.node.create_publisher(Float64MultiArray, "/effort_control/commands",10)
        self.joint_state_sub  =self.node.create_subscription(JointState, "/joint_states", self.joint_state_callback,10)
        self.current_received = np.zeros(4, dtype=np.float32)
        self.state_received = False
        self.carpole_reset_pub = self.node.create_publisher(Bool, "/cartpole/reset",10)
        self.system_ready_sub = self.node.create_subscription(Bool, '/cartpole/ready',self.system_ready_cb, 10)

        self.executor =rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self.system_ready= False

    def system_ready_cb(self,msg):
        self.system_ready= msg.data

    def joint_state_callback(self,msg):
        try:
            self.current_observation = np.array([
                msg.position[0],
                msg.velocity[0],
                bound_angle(msg.position[1]),
                msg.velocity[1]
            ], dtype=np.float32)

            self.state_received=True

        except ValueError:
            pass

    def step(self, action):
        """
        Apply the action to the simulation, wait for the next state, and return the observation, reward, done, and info.
        """

        if not self.state_received:
            time.sleep(0.01)
            return self.current_observation, 0.0, False, {}
        
        eff_msg = Float64MultiArray()
        eff_msg.data =[float(action[0])]
        self.cartpole_eff_pub.publish(eff_msg)
        time.sleep(0.01)
        obs = self.current_observation.copy()
        reward =1.0 

        done =bool(
            obs[0]< -2 or obs[0]> 2 or 
            obs[2] < -math.pi/4 or obs[2] >math.pi/4 or 
            math.fabs(obs[3] >0.3) or math.fabs(obs[1]>1.0))
        
        if math.fabs(obs[1] >1.0) :
            reward =-1.0

        truncated =False
        info ={}

        return obs, reward, done, truncated, info
    
    def reset(self, seed=None, options=None):
        if seed is not None:
            np.random.seed(seed)

        reset_data = Bool()
        reset_data.data =True
        self.carpole_reset_pub.publish(reset_data)

        time.sleep(0.5)

        while not self.system_ready:
            time.sleep(0.001)

        return self.current_observation.copy(), {}
    
    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()


