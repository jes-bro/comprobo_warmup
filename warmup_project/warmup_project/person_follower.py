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

class PersonFollowerNode(Node):

    def __init__(self):
        super().__init__('person_follower_node')
        # Args: interval between invocations of the timer (period), (callback)
        self.create_timer(0.1, self.run_loop) # executes run loop 10x a second - how does callback work exactly
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        # create publisher to publish messages to topic
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.ranges = []
        self.follow_distance = 5

    def process_scan(self, laser_data):
        ranges = laser_data.ranges
        self.ranges = ranges[315:] + ranges[:45]
    
    def move_forward(self, msg):
        msg.linear.x = 0.05
        msg.angular.z = 0.0
        self.pub.publish(msg)
        sleep(0.1)
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
        msg.angular.z = 0.5
        self.pub.publish(msg)
        sleep(0.1)
        msg.angular.z = 0.0
        self.pub.publish(msg)
    
    def turn_right(self, msg):
        msg.angular.z = -0.5
        self.pub.publish(msg)
        sleep(0.1)
        msg.angular.z = 0.0
        self.pub.publish(msg)


    def run_loop(self):
        msg = Twist()

        # Check to see if LIDAR data empty
        if (len(self.ranges) == 0):
            return
        
        closest_point = min(self.ranges)

        closest_point_idx = self.ranges.index(closest_point)

        # Index corresponds to point's angle relative to NEATO
        angular_error = closest_point_idx - 45

        # Adjust angular position (rotate)
        if (not (-5 < angular_error < 5)):
            if (angular_error < 0):
                self.turn_right(msg)
            else:
                self.turn_left(msg)
            return

        # Adjust linear position (translate)
        if (not abs(self.follow_distance - closest_point) > 1):
            if (self.follow_distance - closest_point < 0):
                self.move_forward(msg)
            else:
                self.move_backward(msg)
            return

        sleep(0.5)

def main():
    rclpy.init()
    node = PersonFollowerNode()
    rclpy.spin(node) #infinite loop
    rclpy.shutdown() # when is spin complete


if __name__ == '__main__':
    main()