# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# std publisher dependencies
import rclpy
from rclpy.node import Node
# External dependencies
from nav_msgs.msg import OccupancyGrid 

from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist
import time

class BtRoot():
    ''' Params: Array of childs '''
    def __init__(self, childs):
        self.childs = [*childs]
    def add_child(self, child):
        self.childs.append(child)
    def run(self):
        for child in self.childs:
            # we dont care about the return value    
            ret = child.run() 

class BtSequential():
    ''' Params: Array of childs '''
    def __init__(self,childs):
        self.childs = [*childs]
    def add_child(self, child):
        self.childs.append(child)
    def run(self):
        for child in self.childs:
            ret = child.run()
            if ret == False:
                return False
        return True    
    
class BtRepeatSequential():
    ''' Repeat node that succedes when all the the actions succeded.
        Only accepts one childs at it is a decorator.''' 
    def __init__(self,child ,repeat_number: int):
        self.child = child
        self.repeat_number = repeat_number
    def add_child(self, child):
        self.child.append(child)
    def run(self):
        for i in range(self.repeat_number) :
            ret = self.child.run()
            if ret == False:
                return False
        return True     

class BtActionNode():
    ''' Action nodes links functions to callable nodes 
        params: function, args ''' 
    def __init__(self,child, args):
        self.child = child
        self.args = args

    def run(self):
        ret = self.child(*self.args)
        return ret      
      
class LaserSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 11)
        print('sub created')
        self.subscription  # prevent unused variable warning
        self.msg

    def listener_callback(self, msg):
        self.msg = msg.ranges
        #print("listener callback called")
        #self.get_logger().info('I heard: "%s"' % msg.ranges)    

class TwistPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.i = 0


def stop_movement_fn(pub):
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub.publisher_.publish(msg) 

def spin_fn(pub):
    print("spin called") # Debug 
    msg = Twist()
    # Set linear velocity (m/s) for moving straight
    msg.angular.z = 50.7  # high angular velocity for avoiding getting stuck  
    msg.linear.x  = 0.1 
    pub.publisher_.publish(msg)
    print("message published")
    time.sleep(1.0) # 1.45
    stop_movement_fn(pub)
        

def move_ahead_incremental(pub, sleep_time):
    '''Move at 0.5 in linear X during a sleep time
       param: publisher, sleep_time'''
    print("From move fn",sleep_time) # Debug 
    msg = Twist()
    # Set linear velocity (m/s) for moving straight
    msg.angular.z = 0.0  # No angular velocity for straight movement
    msg.linear.x = 0.5
    pub.publisher_.publish(msg)
    print("message published")
    time.sleep(sleep_time[0])
    stop_movement_fn(pub)


def increment_fn(x, increment):
    x[0] += increment # Modify the value inside the list
    print(x[0])

def not_crashing_fn(lasser_info, increment):
    pass
    # return true or false 

def main(args=None):
    print("sym")
    rclpy.init(args=args)
    lidar_subscriber = LaserSubscriber()
    rclpy.spin(lidar_subscriber)
    twist_publisher = TwistPublisher()
    # Sleep time is used as an incremental counter to move ahead. IE of sleep time values: 2.0, 2.2, 2.4 ...
    # Whit this approach we achive the spiral 
    sleep_time = [2.0]  # function input. 
    #Setting up the BT
    spin = BtActionNode(spin_fn, [twist_publisher])
    move =  BtActionNode(move_ahead_incremental, [twist_publisher,  sleep_time])
    increment = BtActionNode(increment_fn, [sleep_time, 0.30])
    not_crashing =  BtActionNode(not_crashing_fn, [lidar_subscriber.msg]) # Does the value get frozzen? Chek increment fn if it does
    sq1 = BtSequential([move, spin, increment])
    repeat = BtRepeatSequential(sq1, 100)
    root = BtRoot([repeat])
    # Execute the BT
    root.run() 
    print("Returned from root") # We know the node is done # Debug propourses
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_subscriber.destroy_node()
    twist_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
