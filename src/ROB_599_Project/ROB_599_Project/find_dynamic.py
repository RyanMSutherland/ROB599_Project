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
import cv2 as cv

class FindDynamic(Node):
    def __init__(self):
        super().__init__('find_dynamic')

        self.map_listener = self.create_subscription(OccupancyGrid, '/map', self.update_maps, 10)

        self.current_map = None
        self.all_lines = None
        self.max_move_speed = 0.1

    def update_maps(self, occupancy_msg):
        if self.current_map == None:
            self.current_map = occupancy_msg
            return
        
        recieved_map = np.reshape(occupancy_msg.data, (occupancy_msg.info.height, occupancy_msg.info.width))
        recieved_map = (recieved_map*255).astype(np.uint8)

        all_new_lines = cv.HoughLinesP(recieved_map, 1, np.pi/180, 50, minLineLength = 5, maxLineGap = 10)
        recieved_map = cv.cvtColor(recieved_map,cv.COLOR_GRAY2BGR)

        for line in all_new_lines:
            x1, y1, x2, y2 = line[0]
            cv.line(recieved_map, (x1, y1), (x2, y2), (0, 255, 0), 1)

        cv.imwrite('hahahahaha_new.jpg', recieved_map)

        old_map = np.reshape(self.current_map.data, (self.current_map.info.height, self.current_map.info.width))
        old_map = (old_map*255).astype(np.uint8)

        self.all_lines = cv.HoughLinesP(old_map, 1, np.pi/180, 50, minLineLength = 5, maxLineGap = 10)
        old_map = cv.cvtColor(old_map,cv.COLOR_GRAY2BGR)

        for line in self.all_lines:
            x1, y1, x2, y2 = line[0]
            cv.line(old_map, (x1, y1), (x2, y2), (0, 255, 0), 1)
        cv.imwrite('hahahahaha_old.jpg', old_map)

        for new_line in all_new_lines:
            x1_new, y1_new, x2_new, y2_new = new_line[0]
            min_dist = np.inf
            min_location = None
            for old_line in self.all_lines:
                x1_old, y1_old, x2_old, y2_old = old_line[0]
                
                x1_same_dist = np.sqrt((x1_new - x1_old)**2 + (y1_new - y1_old) ** 2)

        self.current_map = occupancy_msg



def main(args = None):
    rclpy.init(args = args)

    find_dunamic = FindDynamic()
    rclpy.spin(find_dunamic)

    find_dunamic.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()