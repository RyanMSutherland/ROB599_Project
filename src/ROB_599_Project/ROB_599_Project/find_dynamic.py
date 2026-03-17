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
from scipy.stats import norm
from skimage import draw
import json

class FindDynamic(Node):
    def __init__(self):
        super().__init__('find_dynamic')

        self.map_listener = self.create_subscription(OccupancyGrid, '/map', self.update_maps, 10)

        self.current_map = None
        self.all_lines = None
        self.max_move_speed = 0.1
        self.move_error = 0.1

        self.robot_size = 0.3

        self.line_probabilities = {}

    def update_maps(self, occupancy_msg):
        if self.current_map == None:
            self.current_map = occupancy_msg

            initial_map = np.reshape(self.current_map.data, (self.current_map.info.height, self.current_map.info.width))
            initial_map = (initial_map*255).astype(np.uint8)

            free_space = (initial_map > 10)
            initial_map[free_space] = 255

            edges = cv.Canny(initial_map, 175, 200)
            self.all_lines = cv.HoughLinesP(edges, 1, np.pi/180, 10, minLineLength = 15, maxLineGap = 5)

            # self.all_lines = cv.HoughLinesP(initial_map, 1, np.pi/180, 50, minLineLength = 5, maxLineGap = 10)

            initial_map = cv.cvtColor(initial_map,cv.COLOR_GRAY2BGR)

            initial_points = np.zeros([len(self.all_lines),4])
            self.all_initial_points = []
            for idx, line in enumerate(self.all_lines):
                x1, y1, x2, y2 = line[0]
                cv.line(initial_map, (x1, y1), (x2, y2), (0, 255, 0), 1)
                initial_points[idx] = [x1, y1, x2, y2]
                self.all_initial_points.append([x1, y1])
                self.all_initial_points.append([x2, y2])
                self.line_probabilities[(x1, y1, x2, y2)] = 0.5
            cv.imwrite('hahahahaha_old.jpg', initial_map)
            print(len(self.all_initial_points))

            print(self.line_probabilities)
            return
        
        recieved_map = np.reshape(occupancy_msg.data, (occupancy_msg.info.height, occupancy_msg.info.width))
        recieved_map = (recieved_map*255).astype(np.uint8)
        
        # full_space = (recieved_map == 1)
        # recieved_map[full_space] = 0
        free_space = (recieved_map > 10)
        recieved_map[free_space] = 255

        edges = cv.Canny(recieved_map, 175, 200)
        all_new_lines = cv.HoughLinesP(edges, 1, np.pi/180, 25, minLineLength = 15, maxLineGap = 5)

        time_difference = (occupancy_msg.header.stamp.sec + occupancy_msg.header.stamp.nanosec * 10 **-9) - (self.current_map.header.stamp.sec + self.current_map.header.stamp.nanosec * 10 **-9)
        self.current_map = occupancy_msg
        

        recieved_map = cv.cvtColor(recieved_map,cv.COLOR_GRAY2BGR)
        cv.imwrite('first_new.jpg', recieved_map)

        new_points = np.zeros([len(all_new_lines),4])
        all_new_points = []
        for idx, line in enumerate(all_new_lines):
            x1, y1, x2, y2 = line[0]
            cv.line(recieved_map, (x1, y1), (x2, y2), (0, 255, 0), 1)
            new_points[idx] = [x1, y1, x2, y2]
            all_new_points.append([x1, y1])
            all_new_points.append([x2, y2])

        cv.imwrite('hahahahaha_new.jpg', recieved_map)

        print(len(new_points))

        # Totally static finder
        all_points = []
        for new_point in all_new_points:
            if new_point in self.all_initial_points:
                # self.get_logger().info(f"Found point: {new_point}, in both images")

                point_1 = np.where((new_points[:, :2] == np.array(new_point)).all(axis=1))
                point_2 = np.where((new_points[:, 2:] == np.array(new_point)).all(axis=1))

                if len(point_1[0]) > 0:    
                    if tuple(new_points[point_1[0][0]]) in self.line_probabilities:
                        all_points.append(new_points[point_1[0][0]])
                        # print(tuple(new_points[point_1[0][0]]))
                        sensor_reading = 0.1
                        prior = self.line_probabilities[tuple(new_points[point_1[0][0]])]
                        self.line_probabilities[tuple(new_points[point_1[0][0]])] = np.clip(sensor_reading * prior / (sensor_reading * prior + (1-sensor_reading) * (1-prior)), a_min = 0.01, a_max = 0.99)

                        new_points = np.delete(new_points, point_1[0][0], axis = 0)

                elif len(point_1[0]) > 0:                    
                    if tuple(new_points[point_2[0][0]]) in self.line_probabilities:
                        all_points.append(new_points[point_2[0][0]])
                        sensor_reading = 0.1
                        prior = self.line_probabilities[tuple(new_points[point_2[0][0]])]
                        self.line_probabilities[tuple(new_points[point_2[0][0]])] = np.clip(sensor_reading * prior / (sensor_reading * prior + (1-sensor_reading) * (1-prior)), a_min = 0.01, a_max = 0.99)

                        new_points = np.delete(new_points, point_2[0][0], axis = 0)

        # print(new_points)
        # print(self.line_probabilities)
        # print(all_points)
        print(len(self.line_probabilities))
        print(len(new_points))

        all_available_points = []
        if len(all_points) > 0:
            stacked_all_points = np.stack(all_points)
            for row in self.line_probabilities:
                if row not in stacked_all_points:
                    all_available_points.append(row)
        else:
            for row in self.line_probabilities:
                all_available_points.append(row)
        
        # print(all_available_points)
        all_distances = np.zeros([len(new_points), len(all_available_points)])
        
        for idx, point in enumerate(new_points):
            initial_distances = all_available_points - point
            row_distances = np.sqrt((initial_distances[:, 0])**2 + (initial_distances[:, 1])**2) + np.sqrt((initial_distances[:, 2])**2 + (initial_distances[:, 3])**2) 
            all_distances[idx] = row_distances
        
        # print(all_distances)
        # print(new_points)
        # print(all_available_points)
        
        # for _ in range(min(len(new_points), len(all_available_points))):
        for _ in range(len(new_points)):
            try:
                flat_index = np.argmin(all_distances)
                row, col = np.unravel_index(flat_index, all_distances.shape)

                new_point = new_points[row]
                old_point = all_available_points[col]

                self.line_probabilities[tuple(new_point)] = self.line_probabilities.pop(tuple(old_point))

                # print(all_distances[row][col])
                # sensor_reading = np.clip(norm.pdf(all_distances[row][col] * self.current_map.info.resolution * time_difference, loc = self.max_move_speed * time_difference, scale = self.move_error * time_difference), a_min = 0.1, a_max = 0.9)
                sensor_reading = np.clip((all_distances[row][col] * self.current_map.info.resolution)/ time_difference, a_min = 0.1, a_max = 0.9)
                # print(all_distances[row][col] * self.current_map.info.resolution * time_difference)
                print(sensor_reading)
                prior = self.line_probabilities[tuple(new_point)]
                self.line_probabilities[tuple(new_point)] = np.clip(sensor_reading * prior / (sensor_reading * prior + (1-sensor_reading) * (1-prior)), a_min = 0.01, a_max = 0.99)

                all_distances = np.delete(all_distances, row, axis = 0)
                all_distances = np.delete(all_distances, col, axis = 1)
                new_points = np.delete(new_points, row, axis = 0)
                all_available_points = np.delete(all_available_points, col, axis = 0)
            except:
                continue

        if len(new_points) > 0:
            print("Added points")
            for points in new_points:
                self.line_probabilities[tuple(points)] = 0.5
        if len(all_available_points) > 0:
            print("Remove points")
            for point in all_available_points:
                _ = self.line_probabilities.pop(tuple(point))
        print(self.line_probabilities)
        
        image = np.copy(recieved_map)
        image[:] = [255, 255, 255]
        # free_space = (self.occupancy_values >= 0) & (self.occupancy_values < 20)
        # image[free_space] = [255, 255, 255]
        # occupied_space = (self.occupancy_values >= 20)
        # image[occupied_space] = [0, 0, 0]

        final_image = np.copy(image)

        x, y= np.ogrid[:image.shape[0], :image.shape[1]]

        for points, prob in self.line_probabilities.items():
            all_points = draw.line(int(points[0]), int(points[1]), int(points[2]), int(points[3]))
            for idx, point in enumerate(all_points[0]):
                dist_sq = ((x - point)*self.current_map.info.resolution)**2 + ((y-all_points[1][idx])*self.current_map.info.resolution)**2
                mask = dist_sq <= ((self.robot_size + prob)/2)**2

                final_image[mask] = [0, 0, 0]

        file_name = f'map_config_save.png'
        cv.imwrite(file_name, np.rot90(final_image))

        # with open('line_probabilities.json', 'w') as f:
        #     json.dump(list(self.line_probabilities.items()), f, indent = 4)

def main(args = None):
    rclpy.init(args = args)

    find_dunamic = FindDynamic()
    rclpy.spin(find_dunamic)

    find_dunamic.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()