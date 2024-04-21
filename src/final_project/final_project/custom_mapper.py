# This is implementation of the custom_mapper node which is responsible
# for mapping the environment using the lidar data and the robot's odometry data.
#
# It subscribes to the /scan and /odom topics and publishes
# the processed map to the /custom_map topic.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Pose

import numpy as np

class CustomMapper(Node):

    def __init__(self):
        super().__init__('custom_mapper')
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.map_updates_publisher = self.create_publisher(OccupancyGridUpdate, '/custom_map_updates', 10)
        
        self.robot_pose = Pose()
        
        self.occupancy_gird_resolution = 0.1 # Align to map_publisher.py
        self.map_origin_x = -10.0  # Meters
        self.map_origin_y = -5.0  # Meters
        
        self.local_grid_width = 30
        self.local_grid_height = 30


    def odom_callback(self, msg):
        self.get_logger().debug('Received odometry data - callback')
        self.robot_pose = msg.pose.pose


    def scan_callback(self, msg):
        self.get_logger().debug('Received scan data - callback')
        
        if self.robot_pose:
            map_update = self.update_map(msg)
            if map_update:
                self.map_updates_publisher.publish(map_update)


    def update_map(self, scan):
        self.get_logger().debug('Updating map with scan data')\

        x_pos = self.robot_pose.position.x
        y_pos = self.robot_pose.position.y
        z_rot = self.robot_pose.orientation.z

        grid_x = int((x_pos - self.map_origin_x) / self.occupancy_gird_resolution)
        grid_y = int((y_pos - self.map_origin_y) / self.occupancy_gird_resolution)

        msg_map_update = OccupancyGridUpdate()
        msg_map_update.header = Header()
        msg_map_update.header.stamp = self.get_clock().now().to_msg()
        msg_map_update.header.frame_id = 'map'
        
        msg_map_update.width  = self.local_grid_width
        msg_map_update.height = self.local_grid_height
        
        msg_map_update.x = grid_x - msg_map_update.width // 2
        msg_map_update.y = grid_y - msg_map_update.height // 2
        
        local_grid = self.get_local_grid(x_pos, y_pos, z_rot, scan)

        msg_map_update.data = local_grid.tolist()

        return msg_map_update
    
    def get_local_grid(self, x, y, rot, scan) -> np.array:
        #return np.array([50] * 9000, dtype=np.int8) # FOR DEBUGGING

        local_grid = np.array([-1] * self.local_grid_width * self.local_grid_height, dtype=np.int8)
        angle_increment = scan.angle_increment # get scan resolution:
        num_measurements = len(scan.ranges)    # get number of measurements:
        angle_min = scan.angle_min             # get the angle of the first measurement

        # For each measurement, calculate the position of the obstacle in the local grid
        for i in range(num_measurements):
            angle = angle_min + i * angle_increment # TODO - Move at the end of the loop
            distance = scan.ranges[i]

            if distance == float('inf'):
                continue # No obstacle detected
            elif distance < scan.range_min or distance > scan.range_max:
                continue # No obstacle detected in a valid range (valiid in terms of the lidar specs)
            else:
                x_obstacle = distance * np.cos(rot + angle)
                y_obstacle = distance * np.sin(rot + angle)

                x_obstacle_in_grid = int(x_obstacle / self.occupancy_gird_resolution + self.local_grid_width / 2)
                y_obstacle_in_grid = int(y_obstacle / self.occupancy_gird_resolution + self.local_grid_height / 2)

                if abs(x_obstacle_in_grid) > self.local_grid_width or \
                   abs(y_obstacle_in_grid) > self.local_grid_height:
                    continue # Check if the measurement is withing local grid, otherwise treat it as invalid

                local_grid[x_obstacle_in_grid + y_obstacle_in_grid * self.local_grid_height] = 100

        return local_grid


def main(args=None):
    rclpy.init(args=args)
    
    custom_mapper = CustomMapper()
    
    rclpy.spin(custom_mapper)
    
    custom_mapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
