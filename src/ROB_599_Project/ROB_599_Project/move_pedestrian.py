import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data

class PedestrianControl(Node):
    def __init__(self):
        super().__init__('pedestrianControl')

        self.publisher_0 = self.create_publisher(Twist, '/pedo/cmd_vel', 10)
        self.odom_sub_0 = self.create_subscription(Odometry, '/pedo/odom', self.update_location, 10)

        # self.publisher_1 = self.create_publisher(Twist, '/pedo_1/cmd_vel', 10)
        # self.odom_sub_1 = self.create_subscription(Odometry, '/pedo_1/odom', self.update_location, 10)

    def update_location(self, msg):
        move = Twist()
        move.linear.x = -0.5
        move.linear.y = 0.0

        self.publisher_0.publish(move)
        # self.publisher_1.publish(move)

def main(args = None):
    rclpy.init(args = args)

    potential_control = PedestrianControl()
    rclpy.spin(potential_control)

    potential_control.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()