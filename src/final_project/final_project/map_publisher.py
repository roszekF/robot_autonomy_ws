import rclpy
import numpy as np
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate


class MapPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/custom_map', 10)
        self.subscriber_map_update = self.create_subscription(OccupancyGridUpdate, '/custom_map_updates', self.map_update_callback, 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.grid = np.array([-1] * 20000, dtype=np.int8)


    def map_update_callback(self, msg): # TODO - think about some better way to update the local area
        self.get_logger().debug('Received map update')
        for i in range(msg.height):
            for j in range(msg.width):
                idx = (msg.y + i) * 200 + (msg.x + j)
                if 0 <= idx < len(self.grid):
                    # update the pixel only if it is defined (not -1)
                    self.grid[idx] = msg.data[i * msg.width + j] if msg.data[i * msg.width + j] != -1 else self.grid[idx]


    def timer_callback(self):
        
        msg = OccupancyGrid()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.map_load_time = self.get_clock().now().to_msg()
        msg.info.resolution = 0.1 # meters per cell
        msg.info.width = 200
        msg.info.height = 100
        msg.info.origin.position.x = -10.0
        msg.info.origin.position.y = -5.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.tolist()

        self.publisher_.publish(msg)
        self.get_logger().debug('Publishing map')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    map_publisher = MapPublisher()

    rclpy.spin(map_publisher)

    map_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
