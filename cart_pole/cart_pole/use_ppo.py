import time ,rclpy
from stable_baselines3 import PPO
from cart_pole.cart_pole_env import CarPoleROS2Env



def main(args=None):
    model = PPO.load("ppo_cartpole_v1.zip")
    env = CarPoleROS2Env()
    obs, _= env.reset()
    time.sleep(1)

    done=False
    while True:
        action, _= model.predict(obs)
        obs,_ , done, _, info = env.step(action)
        time.sleep(0.001)

    env.close()


if __name__ =="__main__":
    main()