
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data
from PIL import Image

class Mapping(Node):
    def __init__(self):
        super().__init__('Mapping')

        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        self.grid_size = 0.032
        self.map_width = 16
        self.map_height = 16
        
        self.image_name = "cave.png"
        with Image.open(self.image_name) as img:
            self.image = np.array(img)
        self.image = np.where(self.image == 255, 0, 100).astype(np.int8)

        map_update_frequency = 5.0
        self.map_update = self.create_timer(map_update_frequency, self.publish_map)



    def publish_map(self):
        msg = OccupancyGrid()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.info.resolution = self.grid_size
        msg.info.width = int(self.map_width/self.grid_size)
        msg.info.height = int(self.map_height/self.grid_size)

        msg.info.origin.position.x = -8.0
        msg.info.origin.position.y = -8.0
        # self.get_logger().info(f"IMAGE: {self.image}")
        msg.data = np.flip(self.image, axis = 0).flatten()

        self.map_publisher.publish(msg)
        self.get_logger().info("Published map!")
        
        

def main(args = None):
    rclpy.init(args = args)

    mapping = Mapping()
    rclpy.spin(mapping)

    mapping.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()