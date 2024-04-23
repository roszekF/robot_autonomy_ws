import threading

import rclpy
import numpy as np
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class MapPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/custom_map', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.width = 200
        self.height = 100
        # [0,0] is bottom left corner
        self.grid = np.ones((self.width,self.height), dtype=np.int8) * -1 # -1 is unknown, 0 is free, 100 is occupied

        self.odom_msg_lock = threading.Lock()
        self.latest_odom_msg = None

        self.lidar_msg_lock = threading.Lock()
        self.latest_lidar_msg = None

        self.map_message = OccupancyGrid()
        self.map_message.header.frame_id = 'map'
        self.map_message.info.resolution = 0.1 # m per cell
        self.map_message.info.width = self.width
        self.map_message.info.height = self.height
        self.map_message.info.origin.position.x = -10.0
        self.map_message.info.origin.position.y = -5.0
        self.map_message.info.origin.position.z = 0.0
        quat = quaternion_from_euler(0, 0, 0)
        self.map_message.info.origin.orientation.x = quat[0]
        self.map_message.info.origin.orientation.y = quat[1]
        self.map_message.info.origin.orientation.z = quat[2]
        self.map_message.info.origin.orientation.w = quat[3]


    def timer_callback(self):
        msg = self.map_message

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.map_load_time = self.get_clock().now().to_msg()

        # get the robot's position
        with self.odom_msg_lock:
            if self.latest_odom_msg is None:
                return
            robot_x = self.latest_odom_msg.pose.pose.position.x
            robot_y = self.latest_odom_msg.pose.pose.position.y
            _, _, robot_yaw = euler_from_quaternion([self.latest_odom_msg.pose.pose.orientation.x,
                                                self.latest_odom_msg.pose.pose.orientation.y,
                                                self.latest_odom_msg.pose.pose.orientation.z,
                                                self.latest_odom_msg.pose.pose.orientation.w])


        # calculate robot position
        robot_grid_x = int((robot_x - msg.info.origin.position.x) / msg.info.resolution)
        robot_grid_y = int((robot_y - msg.info.origin.position.y) / msg.info.resolution)

        # get the lidar data
        with self.lidar_msg_lock:
            if self.latest_lidar_msg is None:
                return
            lidar_angle_min = self.latest_lidar_msg.angle_min
            lidar_angle_max = self.latest_lidar_msg.angle_max
            lidar_angle_increment = self.latest_lidar_msg.angle_increment
            lidar_ranges = self.latest_lidar_msg.ranges

        for i, range in enumerate(lidar_ranges):
            draw_occupied = True
            # if range is inf, draw a line up to 3.5m
            if range == float('inf'):
                draw_occupied = False
                range = 3.5

            # calculate obstacle position
            angle = lidar_angle_min + i * lidar_angle_increment + robot_yaw
            x = robot_x + range * np.cos(angle)
            y = robot_y + range * np.sin(angle)
            obstacle_grid_x = int((x - msg.info.origin.position.x) / msg.info.resolution)
            obstacle_grid_y = int((y - msg.info.origin.position.y) / msg.info.resolution)

            # draw a "free" line from the robot to the obstacle
            self.draw_line(robot_grid_x, robot_grid_y, obstacle_grid_x, obstacle_grid_y)

            # draw the obstacle
            if draw_occupied and obstacle_grid_x >= 0 and obstacle_grid_x < self.width and obstacle_grid_y >= 0 and obstacle_grid_y < self.height:
                self.grid[obstacle_grid_x, obstacle_grid_y] = 100

        msg.data = self.grid.flatten(order='F').tolist()
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing map')

    def odom_callback(self, msg):
        with self.odom_msg_lock:
            self.latest_odom_msg = msg

    def scan_callback(self, msg):
        with self.lidar_msg_lock:
            self.latest_lidar_msg = msg

    def draw_line(self, x0, y0, x1, y1):
        # Bresenham's line algorithm
        dx = abs(x1 - x0)
        if x0 < x1:
            sx = 1
        else:
            sx = -1

        dy = -abs(y1 - y0)
        if y0 < y1:
            sy = 1
        else:
            sy = -1

        err = dx + dy

        while True:
            if x0 >= 0 and x0 < self.width and y0 >= 0 and y0 < self.height:
                self.grid[x0, y0] = 0
            else:
                break

            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                if x0 == x1:
                    break
                err = err + dy
                x0 = x0 + sx
            if e2 <= dx:
                if y0 == y1:
                    break
                err = err + dx
                y0 = y0 + sy



def main(args=None):
    rclpy.init(args=args)

    map_publisher = MapPublisher()

    rclpy.spin(map_publisher)

    map_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
