import gymnasium as gym
from gymnasium.spaces import Tuple, Box

import numpy as np

import rclpy
from rclpy.node import Node

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode

from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import Image

import numpy as np

import ray
from ray.tune.registry import register_env
import time
from torchvision.transforms import ToTensor

class Environment(Node, gym.Env):

    def __init__(self):
        super().__init__('sim_env')

        self.pose_publisher_ = self.create_publisher(Float32MultiArray, "/set_pose", 10)
        self.speed_publisher_ = self.create_publisher(Float32MultiArray, "/speed", 10)

        self.disarm_publisher_ = self.create_publisher(Float32, "/disarm", 10)
        self.arm_publisher_ = self.create_publisher(Float32, "/arm", 10)


        self.score = None
        self.score_subscriber = self.create_subscription(Float32, "/score", self.score_callback, 10)

        self.camera_data = None
        self.camera_subscription = self.create_subscription(Image, '/rgbd_camera/depth_image', self.camera_callback, 10)

        self.action_space = Box(low=np.array([-10.0, -10.0, 0.1, -3.14], dtype=np.float32), high=np.array([10.0, 10.0, 10.0, 3.14], dtype=np.float32), dtype=np.float32)
        self.observation_space = Box(low=np.full((3,640,480), 0, dtype=np.float32), high=np.full((3,640,480), 1, dtype=np.float32), dtype=np.float32)
        
    def _get_obs(self):
        new_arr = np.reshape(np.array(np.array(self.camera_data) / 255.0, dtype=np.float32), (640,480,4))[:,:,:3].reshape(3, 640, 480)
        print(new_arr.shape)
        return new_arr

    def score_callback(self, msg):
        self.score = msg.data

    def camera_callback(self, msg):
        data = msg.data
        self.camera_data = data

    def disarm(self):
        msg = Float32()
        msg.data = 0.0
        self.disarm_publisher_.publish(msg)

    def arm(self):
        msg = Float32()
        msg.data = 0.0
        self.arm_publisher_.publish(msg)

    def reset(self, *, seed=None, options=None):
        #self.disarm()

        speed_msg = Float32MultiArray()
        speed_msg.data = [0.0, 0.0, 0.1, 0.0]
        self.speed_publisher_.publish(speed_msg)

        time.sleep(1)

        rover_pos = [0.0, 2.0, 0.2, 0.0, 0.0, 1.0, 1.0]
        drone_pos = [0.0, 0.0, 0.1, 0.0, 0.0, 1.0, 1.0]

        poses = rover_pos + drone_pos

        pos_msg = Float32MultiArray()
        pos_msg.data = poses

        self.pose_publisher_.publish(pos_msg)
        self.get_logger().info('Sent reset')

        time.sleep(1)

        #self.arm()

        observation = self._get_obs()
        info = {}

        return observation, info
    
    def step(self, action):
        self.get_logger().info(f'Step: {action}')
        speed_msg = Float32MultiArray()
        speed_msg.data = action.tolist()
        self.speed_publisher_.publish(speed_msg)

        observation = self._get_obs()
        reward = np.abs(self.score - 1.0)
        terminated = np.abs(self.score - 1.0) < 0.5
        truncated = self.score > 2.5
        info = {}

        return observation, reward, terminated, truncated, info

    def render(self):
        pass

import threading
from ray.rllib.algorithms.dreamer import DreamerConfig

def ros_run(node):
    print("starting node")
    rclpy.spin(node)
    sim_env.destroy_node()
    print("shutdown ros node")
    rclpy.shutdown()

def set_model(config):
    sim_env = Environment()
    x = threading.Thread(target=ros_run, args=(sim_env,))
    x.start()
    time.sleep(2)
    ray.rllib.utils.check_env(sim_env)
    return sim_env


def main(args=None):
    rclpy.init(args=args)
    ray.init(num_gpus=1)

    register_env("sim_env", set_model)
    print("Registered environment")


    config = DreamerConfig().training(gamma=0.9, lr=0.01).resources(num_gpus=1).framework(framework='torch')
    algo = config.build(env="sim_env") 
    print("Configured model")

    while True:
        print("Training...")
        algo.train()


if __name__ == '__main__':
    main()
     
