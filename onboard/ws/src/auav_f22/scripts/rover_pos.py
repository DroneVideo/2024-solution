import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Float32, Bool
from px4_msgs.msg import VehicleOdometry
from tf2_msgs.msg import TFMessage
import numpy as np

class RoverSubscriber(Node):

    def __init__(self):
        super().__init__('rover_subscriber')
        self.rover_subscription = self.create_subscription(TFMessage, '/world/trial_1_world/dynamic_pose/info', self.rover_callback, 10)
        self.score_publisher = self.create_publisher(Float32, 'score', 10)

        self.rover_position = None
        self.drone_position = None
        self.sum = 0
        self.samples = 0


    def rover_callback(self, odom):
        self.rover_position = [odom.transforms[0].transform.translation.x, odom.transforms[0].transform.translation.y, odom.transforms[0].transform.translation.z]
        self.drone_position = [odom.transforms[2].transform.translation.x, odom.transforms[2].transform.translation.y, odom.transforms[2].transform.translation.z]

        distance = np.linalg.norm(np.array([
            self.rover_position[0] - self.drone_position[0],
            self.rover_position[1] - self.drone_position[1],
            self.rover_position[2] - self.drone_position[2]]))
        inst_score = 0
        if (distance <  5):
            inst_score = 1 - np.abs(distance - 1)/4
        self.sum += inst_score
        self.samples += 1
        self.score = self.sum/self.samples
        #self.get_logger().info('distance: {}, inst score: {}, sum: {} samples: {}, score: {}'.format(distance, inst_score, self.sum, self.samples, self.score))
        self.score_publisher.publish(Float32(data=distance))
        

def main(args=None):
    rclpy.init(args=args)
    rover_subscriber = RoverSubscriber()
    rclpy.spin(rover_subscriber)
    rover_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()