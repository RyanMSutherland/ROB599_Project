
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
import pickle
import cv2 as cv

class Mapping(Node):
    def __init__(self):
        super().__init__('Mapping')

        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        self.grid_size = 0.032
        self.map_width = 16
        self.map_height = 16

        map_update_frequency = 5.0
        self.map_update = self.create_timer(map_update_frequency, self.publish_map)
        self.count = 0

        self.robot_size = 0.3

        with open('empty.world.pkl', 'rb') as f:
            occupancy_grid_data = pickle.load(f)

        print(occupancy_grid_data.header)
        print(occupancy_grid_data.info)

        initial_map = np.reshape(occupancy_grid_data.data, (occupancy_grid_data.info.height, occupancy_grid_data.info.width))
        initial_map = (initial_map*255).astype(np.uint8)

        free_space = (initial_map > 10)
        initial_map[free_space] = 255

        cv.imwrite('nav2_map_normal.jpg', initial_map)

        final_image = np.copy(initial_map)
        final_image[:] = 255

        x, y= np.ogrid[:initial_map.shape[0], :initial_map.shape[1]]

        for point_x in range(0, int(occupancy_grid_data.info.height)):
            for point_y in range(0, int(occupancy_grid_data.info.width)):
                if initial_map[point_x][point_y] == 255:
                    dist_sq = ((x - point_x)*0.05)**2 + ((y-point_y)*0.05)**2
                    mask = dist_sq <= (self.robot_size/2)**2

                    final_image[mask] = 0
    
        cv.imwrite('nav2_map_config.jpg', np.flipud(final_image))


    def publish_map(self):
        msg = OccupancyGrid()

        self.image_name = f"cave_{0}.png"
        with Image.open(self.image_name) as img:
            self.image = np.array(img)
        self.image = np.where(self.image == 255, 0, 100).astype(np.int8)

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
        self.count += 1
        
        

def main(args = None):
    rclpy.init(args = args)

    mapping = Mapping()
    rclpy.spin(mapping)

    mapping.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()