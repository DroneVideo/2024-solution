import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from px4_msgs.msg import VehicleOdometry

import numpy as np

class DroneState(Node):

    def __init__(self):
        super().__init__('drone_state')
        self.camera_subscription = self.create_subscription(Image, '/rgbd_camera/depth_image', self.camera_callback, 10)
        self.pos_subscription = self.create_subscription(VehicleOdometry, '/VehicleOdometry_PubSubTopic', self.pos_callback, 10)

        self.camera_data = None
        self.pos_data = None

    def camera_callback(self, msg):
        data = msg.data
        self.camera_data = data

    def pos_callback(self, msg):
        self.pos_data = np.array([msg.x, msg.y, msg.z, msg.q, msg.vx, msg.vy, msg.vz])
        self.get_logger().info(str(self.pos_data))

def main(args=None):
    rclpy.init(args=args)
    drone_state = DroneState()
    rclpy.spin(drone_state)
    drone_state.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()