import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
from tf_transformations import euler_from_quaternion
import cv2 as cv
from skimage import draw
from shapely.geometry import LineString

class FindDynamic2(Node):
    def __init__(self):
        super().__init__('find_dynamic_2')

        self.laser_scan = self.create_subscription(LaserScan, '/robot_0/base_scan', self.laser_update, 1)
        self.odom_subscriber = self.create_subscription(Odometry, '/robot_0/ground_truth', self.update_location, 1)
        self.publisher = self.create_publisher(Twist, '/robot_0/cmd_vel', 10)

        self.grid_size = 0.05
        self.map_width = 20
        self.map_height = 20
        self.start_position = [int(self.map_width/2), int(self.map_height/2)]
        self.angle_max_thresh = 1.0
        self.robot_size = 1.0

        self.odom = None
        print("Starting")

        self.line_dist_thresh = 3

        self.all_lines = {}

    def update_location(self, odom_msg):
        self.odom = odom_msg
        print("Odom recieved")

        move = Twist()
        move.linear.x = .2
        # self.publisher.publish(move)
    
    def laser_update(self, laser_msg):
        if self.odom == None:
            return
        
        print("Laser recieved")
        current_map = np.zeros([int(self.map_width/self.grid_size), int(self.map_height/self.grid_size)])
        current_position = self.odom.pose.pose
        orientation_list = [current_position.orientation.x, current_position.orientation.y, current_position.orientation.z, current_position.orientation.w]
        current_grid_location = np.array([current_position.position.x + self.start_position[0], current_position.position.y + self.start_position[1]])

        for idx, dist in enumerate(laser_msg.ranges):
            if dist == laser_msg.range_max:
                continue
                
            current_angle = laser_msg.angle_increment * idx + laser_msg.angle_min + euler_from_quaternion(orientation_list)[-1]
            final_grid_location = np.array([np.cos(current_angle)*dist + current_grid_location[0], np.sin(current_angle)*dist + current_grid_location[1]])

            current_map[round(final_grid_location[1]/self.grid_size)][round(final_grid_location[0]/self.grid_size)] = 255

        current_map = (current_map).astype(np.uint8)
        edges = cv.Canny(current_map, 175, 200)
        all_new_lines = cv.HoughLinesP(edges, 1, np.pi/180, 25, minLineLength = 10, maxLineGap = 5)
        current_map = cv.cvtColor(current_map,cv.COLOR_GRAY2BGR)

        # print(all_new_lines)
        cv.imwrite('turd_nugget_supreme.jpg', current_map)

        for idx, line in enumerate(all_new_lines):
            found_within = False
            x1, y1, x2, y2 = line[0]
            pre_line = line
            new_line = LineString([(x1, y1), (x2, y2)])
            # print(line[0])
            nearest_distance = np.inf
            nearest_idx = 0
            for idx, saved_line in enumerate(self.all_lines):
                saved_line = np.array(saved_line)
                line_segment = LineString([(saved_line[0], saved_line[1]), [saved_line[2], saved_line[3]]])
                # print(f"Distance: {line_segment.distance(new_line)}")
                dist = line_segment.distance(new_line)
                if dist < nearest_distance:
                    nearest_distance = dist
                    nearest_idx = idx
                if line_segment.distance(new_line) <= self.line_dist_thresh:
                    found_within = True
                    # print(f"Found same line at: {line[0]}")
                    new_best, best_idx = self.find_longest_line(saved_line, line[0])
                    prior = self.all_lines[tuple(saved_line)]
                    self.all_lines[tuple(new_best)] = self.all_lines.pop(tuple(saved_line))
                    if best_idx == 0:
                        sensor_reading = 0.7
                    else:
                        sensor_reading = 0.6
                    self.all_lines[tuple(new_best)] = np.clip(sensor_reading * prior / (sensor_reading * prior + (1-sensor_reading) * (1-prior)), a_min = 0.01, a_max = 0.99)
                    break
            
            if nearest_distance < 15.0 and nearest_distance > 3.0 and not found_within:
                line, prior = list(self.all_lines.items())[nearest_idx]
                sensor_reading = np.clip(nearest_distance*self.grid_size, a_min = 0.2, a_max=0.8)
                print(f"Dynamic object, distance: {nearest_distance}. Closest line: {line}, Prior: {prior}, sensor reading: {sensor_reading}, preline: {pre_line}")
                self.all_lines[tuple(line)] = np.clip(sensor_reading * prior / (sensor_reading * prior + (1-sensor_reading) * (1-prior)), a_min = 0.01, a_max = 0.99)
                
                self.all_lines[tuple(pre_line[0])] = self.all_lines.pop(tuple(line))
                print(f"Value updated to : {self.all_lines[tuple(pre_line[0])]}")

            elif not found_within:
                self.all_lines[x1, y1, x2, y2] = 0.5
                print(f"New line: {[x1, y1, x2, y2]}")
                # print(self.all_lines)
            # cv.line(current_map, (x1, y1), (x2, y2), (0, 255, 0), 1)
            # new_points[idx] = [x1, y1, x2, y2]
            # all_new_points.append([x1, y1])
            # all_new_points.append([x2, y2])
        
        print(self.all_lines)

        for idx, line in enumerate(self.all_lines):
            line = np.array(line)
            # print(f"New LINE: {line}")
            cv.line(current_map, (line[0], line[1]), (line[2], line[3]), (0, 255, 0), 1)
        
        cv.imwrite('new_first_new_hehe.jpg', np.flipud(current_map))

        self.get_logger().info(f"Saved image")

        final_image = np.copy(current_map)
        final_image[:] = [255, 255, 255]
        x, y= np.ogrid[:final_image.shape[0], :final_image.shape[1]]

        for points, prob in self.all_lines.items():
            all_points = draw.line(int(points[0]), int(points[1]), int(points[2]), int(points[3]))
            # if prob > 0.4:
            for idx, point in enumerate(all_points[0]):
                dist_sq = ((x - point)*self.grid_size)**2 + ((y-all_points[1][idx])*self.grid_size)**2
                mask = dist_sq <= ((self.robot_size + (1-prob))/2)**2

                final_image[mask] = [0, 0, 0]
        
        file_name = f'map_config_new_hercules.png'
        cv.imwrite(file_name, np.rot90(final_image))

    def find_longest_line(self, line_1, line_2):
        line_combos = []
        line_combos.append([line_1[0], line_1[1], line_1[2], line_1[3]])
        line_combos.append([line_2[0], line_2[1], line_1[2], line_1[3]])
        line_combos.append([line_1[0], line_1[1], line_2[0], line_2[1]])
        line_combos.append([line_2[2], line_2[3], line_1[2], line_1[3]])
        line_combos.append([line_1[0], line_1[1], line_2[2], line_2[3]])
        line_combos.append([line_2[0], line_2[1], line_2[2], line_2[3]])

        best_dist = 0
        best_idx = 0
        best_angle = None
        for idx, line in enumerate(line_combos):
            dist = np.sqrt((line[0] - line[2])**2 + (line[1] - line[3])**2)
            m = (line[3] - line[1])/(line[2] - line[0])
            if dist > best_dist:
                try:
                    # print(f"Best angle: {best_angle}, m: {m}")
                    angle_between = np.arctan2((1 + best_angle * m), (best_angle - m))
                    # print(f"Angle between: {angle_between}, m: {m}")
                    if abs(angle_between) <= self.angle_max_thresh:
                        # best_angle = m
                        best_dist = dist
                        best_idx = idx
                except:
                    # print("ERROR")
                    best_angle = m
                    best_dist = dist
                    best_idx = idx
        # print(f"Best Line: {line_1}, New Line: {line_2}, Best Index: {best_idx}")
        return line_combos[best_idx], best_idx

def main(args = None):
    rclpy.init(args = args)

    find_dynamic_2 = FindDynamic2()
    rclpy.spin(find_dynamic_2)

    find_dynamic_2.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()