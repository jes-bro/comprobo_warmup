import rclpy
from rclpy.node import Node # Import super/base class required to create ROS Node
# Topics are tight so they expect a type of data to go over a topic
# Topic is channel 
# Will stuff break if you use different msg types
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import tty
import select
import sys
import termios
from time import sleep

class ObstacleAvoiderNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoider_node')
        # Args: interval between invocations of the timer (period), (callback)
        self.create_timer(0.1, self.run_loop) # executes run loop 10x a second - how does callback work exactly
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        # create publisher to publish messages to topic
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.ranges = []
        self.distance_threshold = 1
        self.turning_right = True

    def process_scan(self, laser_data):
        ranges = laser_data.ranges
        ranges_list = list(ranges)
        # print(f"ranges list: {ranges_list}")
        ranges_left = ranges_list[0:45]
        ranges_right = ranges_list[314:359]

        ranges_left_reversed = []
        ranges_right_reversed = []

        for i in reversed(range(0, 45)):
            ranges_left_reversed.append(ranges_left[i])
            ranges_right_reversed.append(ranges_right[i])

        self.ranges = ranges_left_reversed + ranges_right_reversed
    
    def move_forward(self, msg):
        msg.linear.x = 0.05
        msg.angular.z = 0.0
        self.pub.publish(msg)
        sleep(0.3)
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)
    
    def move_backward(self, msg):
        msg.linear.x = -0.05
        msg.angular.z = 0.0
        self.pub.publish(msg)
        sleep(0.1)
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def turn_left(self, msg):
        self.turning_right = False
        msg.angular.z = 0.1
        self.pub.publish(msg)
        sleep(0.2)
        msg.angular.z = 0.0
        self.pub.publish(msg)
    
    def turn_right(self, msg):
        self.turning_right = True
        msg.angular.z = -0.1
        self.pub.publish(msg)
        sleep(0.2)
        msg.angular.z = 0.0
        self.pub.publish(msg)


    def run_loop(self):
        msg = Twist()
        print("loop")
        # Check to see if LIDAR data empty
        if (len(self.ranges) == 0):
            print("Empty!")
            return
        
        ranges_copy = self.ranges
        closest_point = min(ranges_copy)
        print(f"ranges: {ranges_copy}")
        print(f"closest point: {closest_point}")

        closest_point_idx = ranges_copy.index(closest_point)

        # Index corresponds to point's angle relative to NEATO
        angular_error = closest_point_idx - 45
        print(f"angular error: {angular_error}")

        if abs(self.distance_threshold - closest_point) < 1:
            if (angular_error > 0):
                self.turn_left(msg)
            else:
                self.turn_right(msg)

        self.move_forward(msg)
        sleep(0.5)

def main():
    rclpy.init()
    node = ObstacleAvoiderNode()
    rclpy.spin(node) #infinite loop
    rclpy.shutdown() # when is spin complete


if __name__ == '__main__':
    main()