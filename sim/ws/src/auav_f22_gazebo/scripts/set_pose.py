import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

import os

class SimControl(Node):

    def __init__(self):
        super().__init__('sim_control')

        self.set_pose_subscriber_ = self.create_subscription(Float32MultiArray, '/set_pose', self.set_pose, 10)

    def set_pose(self, msg):
        vals = msg.data
        rover_pos = vals[0:7]
        drone_pos = vals[7:14]

        rover_command = f'ign service -s /world/trial_1_world/set_pose --reqtype ignition.msgs.Pose  --reptype ignition.msgs.Boolean  --timeout 300 --req \'name: \"rover\", position: {{x: {rover_pos[0]}, y: {rover_pos[1]}, z: {rover_pos[2]}}}, orientation: {{x: {rover_pos[3]}, y: {rover_pos[4]}, z: {rover_pos[5]}, w: {rover_pos[6]}}}\''
        os.system(rover_command)
        self.get_logger().info(f'Sent command: {rover_command}')

        drone_command = f'ign service -s /world/trial_1_world/set_pose --reqtype ignition.msgs.Pose  --reptype ignition.msgs.Boolean  --timeout 300 --req \'name: \"drone\", position: {{x: {drone_pos[0]}, y: {drone_pos[1]}, z: {drone_pos[2]}}}, orientation: {{x: {drone_pos[3]}, y: {drone_pos[4]}, z: {drone_pos[5]}, w: {drone_pos[6]}}}\''
        os.system(drone_command)
        self.get_logger().info(f'Sent command: {drone_command}')

def main(args=None):
    rclpy.init(args=args)
    sim_control = SimControl()
    print('Starting node...')
    rclpy.spin(sim_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sim_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()