from cart_pole.cart_pole_env import CarPoleROS2Env
from stable_baselines3 import PPO
from ament_index_python.packages import get_package_share_directory


def main(args =None):
    pakd_nanme = "cart_pole"
    model_pkg_path = get_package_share_directory(package_name=pakd_nanme) +"/"

    env = CarPoleROS2Env()
    model = PPO(
        policy="MlpPolicy",
        env=env,
        verbose=1,
        learning_rate=0.01,          # learning rate
        n_steps=256,                 # number of steps per update
        batch_size=16,               # minibatch size
        n_epochs=10,                 # optimization epochs per update
        gamma=0.99,                  # discount factor
        gae_lambda=0.95,             # GAE lambda
        clip_range=0.2,              # PPO clipping
        ent_coef=0.01,               # entropy coefficient
        vf_coef=0.5,                 # value function coefficient
        max_grad_norm=0.5)
    model.learn(total_timesteps=50, progress_bar=True)
    model.save(model_pkg_path+"ppo_cartpole_v1")

    env.close()

if __name__ =="__main__":
    main()