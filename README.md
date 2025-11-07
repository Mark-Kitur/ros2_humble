1. CartPole – Reinforcement Learning Control

Goal: Balance a cart-pole system using Reinforcement Learning (PPO).

Implemented a custom ROS 2 environment compatible with OpenAI Gym and rclpy.

Trained the controller using Proximal Policy Optimization (PPO) to maintain balance under disturbances.

Designed launch files to spawn the system in Gazebo and observe performance in real time.

Learned how to integrate machine learning with ROS 2 and how RL agents interface with simulation environments.

Key learning:

Reinforcement Learning can autonomously learn control laws through trial and error, and ROS 2 makes it easy to connect these algorithms to simulated or real robots.


2. Navigation – Differential Drive Robot

Goal: Implement navigation for a differential-drive mobile robot.

Used Nav2 (ROS 2 Navigation Stack) with custom launch and configuration files.

Built and tuned AMCL, EKF localization, and SLAM configurations.

Created world maps and robot URDFs for path planning and obstacle avoidance.

Learned how ROS 2 manages TF transforms, sensor fusion, and path planning pipelines.

Key learning:

Navigation in ROS 2 depends heavily on understanding differential drive kinematics — how linear and angular velocities control wheel motion and robot trajectories.


3. Ornithopter – Flapping Wing Robot

Goal: Simulate and control a bio-inspired flapping-wing aircraft.

Modeled the ornithopter’s multi-link wings in Gazebo using URDF and STL meshes.

Implemented a ROS 2 control node to generate periodic wing motion and monitor flight dynamics.

Explored challenges in flapping-wing aerodynamics and control stability.

Integrated the design into the PX4 ecosystem for future autopilot testing.

Key learning:

Flapping-wing flight control requires precise synchronization between mechanical design and actuation patterns — ROS 2 makes experimentation fast and modular.


Through this workspace, I learned the foundations of robot autonomy — from basic differential drive navigation to intelligent control using reinforcement learning.
I also explored advanced bio-inspired robotics through the ornithopter project.
Each package built upon the last, connecting traditional control theory, machine learning, and physical simulation in ROS 2.


Mark Kimutai Kitur
Mechatronic Engineering Student
Dedan Kimathi University of Technology