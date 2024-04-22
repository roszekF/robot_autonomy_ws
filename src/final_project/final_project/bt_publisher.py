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

import rclpy
import time

from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


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
    ''' params: function, args ''' 
    def __init__(self,child, args):
        self.child = child
        self.args = args

    def run(self):
        try:
            ret = self.child(*self.args)
        except:  
            ret = False 
        return ret        


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.i = 0

def stop_fn(pub):
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub.publisher_.publish(msg) 

def spin_fn(pub):
    print("spin called")
    msg = Twist()
    # Set linear velocity (m/s) for moving straight
    msg.angular.z = 50.7  # No angular velocity for straight movement
    msg.linear.x  = 0.1  # adjust this value as needed

    pub.publisher_.publish(msg)
    print("message published")
    time.sleep(1.0) # 1.45
    stop_fn(pub)
        

def move_ahead_incremental(pub, sleep_time):
    '''Move at 0.2 in linear X during a sleep time
       param: publisher, sleep_time'''
    print("From move fn",sleep_time)
    msg = Twist()
    # Set linear velocity (m/s) for moving straight
    msg.angular.z = 0.0  # No angular velocity for straight movement
    msg.linear.x = 0.5
    pub.publisher_.publish(msg)
    print("message published")
    time.sleep(sleep_time[0])
    stop_fn(pub)


def increment_fn(x, increment):
    x[0] += increment # Modify the value inside the list
    print(x[0])

def main(args=None):
    rclpy.init(args=args)
    
    minimal_publisher = MinimalPublisher()
    sleep_time = [2.0]  # function input. 

    spin = BtActionNode(spin_fn, [minimal_publisher])
    move =  BtActionNode(move_ahead_incremental, [minimal_publisher,  sleep_time])
    increment = BtActionNode(increment_fn, [sleep_time, 0.30])
    sq1 = BtSequential([move, spin, increment])
    repeat = BtRepeatSequential(sq1, 100)
    root = BtRoot([repeat])
    root.run()

    print("Returned from root")
    rclpy.spin(minimal_publisher)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
