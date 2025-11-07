import rclpy
import numpy as np
import time, math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix
from gymnasium import spaces, Env


class OrniEnv(Env, Node):
    """Gazebo + ROS2 Gym environment for an ornithopter."""

    def __init__(self):
        # Initialize both Gym Env and ROS2 Node
        Env.__init__(self)
        Node.__init__(self, 'orni_env')

        # ---- Spaces ----
        # Observation: [latitude, longitude, altitude]
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32
        )

        # Actions: [left_wing, right_wing, tail_left, tail_right]
        wing_lim = 1.27  # radians
        tail_lim = 0.8
        self.action_space = spaces.Box(
            low=np.array([-wing_lim, -wing_lim, -tail_lim, -tail_lim], dtype=np.float32),
            high=np.array([wing_lim, wing_lim, tail_lim, tail_lim], dtype=np.float32),
            dtype=np.float32,
        )

        # ---- ROS interfaces ----
        self.pos_sub = self.create_subscription(
            NavSatFix, '/ornithopter/gps/fix', self.pos_callback, 10
        )
        self.wing_pub = self.create_publisher(Float64MultiArray, '/wing_controller/commands', 10)
        self.tail_pub = self.create_publisher(Float64MultiArray, '/tail_controller/commands', 10)

        # ---- State ----
        self.current_pos = np.zeros(3, dtype=np.float32)
        self.target_pos = np.array([0.00040, 0.00033, 6.5], dtype=np.float32)  # hover target
        self.max_bounds = np.array([0.0045, 0.0038, 11.0])  # 10x10x10m roughly
        self.min_bounds = np.array([0.00035, 0.00028, 1.0])

        # smoothing
        self.prev_action = np.zeros(4, dtype=np.float32)
        self.alpha = 0.2  # smoothing factor

        self.start_time = time.time()

    # ---------------------------------------------------------------------
    def pos_callback(self, msg):
        self.current_pos = np.array([msg.latitude, msg.longitude, msg.altitude], dtype=np.float32)

    # ---------------------------------------------------------------------
    def reset(self, *, seed=None, options=None):
        """Resets the environment and returns initial observation."""
        super().reset(seed=seed)
        self.prev_action[:] = 0.0
        self.start_time = time.time()
        # Optionally reset bird pose in Gazebo (via service call)
        obs = self.current_pos.copy()
        return obs, {}

    # ---------------------------------------------------------------------
    def step(self, action):
        """Apply control, observe new state, compute reward."""
        # Smooth action
        smoothed = (1 - self.alpha) * self.prev_action + self.alpha * np.array(action)
        self.prev_action = smoothed

        # Publish to controllers
        wing_msg = Float64MultiArray()
        wing_msg.data = [float(smoothed[0]), float(smoothed[1])]
        self.wing_pub.publish(wing_msg)

        tail_msg = Float64MultiArray()
        tail_msg.data = [float(smoothed[2]), float(smoothed[3])]
        self.tail_pub.publish(tail_msg)

        # Wait for motion to take effect
        time.sleep(0.05)

        obs = self.current_pos.copy()

        # --- Compute reward ---
        pos_err = np.linalg.norm(obs - self.target_pos)
        within_bounds = np.all(obs < self.max_bounds) 
        reward = -pos_err
        done = not within_bounds

        info = {"pos_error": pos_err}
        return obs, reward, done, False, info

    # ---------------------------------------------------------------------
    def close(self):
        self.destroy_node()


# -------------------------------------------------------------------------
# Example usage
if __name__ == "__main__":
    rclpy.init()
    env = OrniEnv()

    obs, _ = env.reset()
    for _ in range(500):
        action = env.action_space.sample()
        obs, reward, done, _, info = env.step(action)
        print(f"Obs={obs}, Reward={reward:.3f}")
        if done:
            print("Out of bounds, resetting...")
            obs, _ = env.reset()
    env.close()
    rclpy.shutdown()
