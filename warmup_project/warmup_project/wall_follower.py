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

class WallFollowerNode(Node):

    def __init__(self):
        super().__init__('square_node')
        # Args: interval between invocations of the timer (period), (callback)
        self.create_timer(0.1, self.run_loop) # executes run loop 10x a second - how does callback work exactly
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        # create publisher to publish messages to topic
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.line_1_distance = 0
        self.line_2_distance = 1

    def process_scan(self, laser_data):
        ranges = laser_data.ranges
        self.line_1_distance = ranges[135]
        self.line_2_distance = ranges[45]
        print("-------------------------")

    
    def move_forward(self, msg):
        msg.linear.x = 0.05
        msg.angular.z = 0.0
        self.pub.publish(msg)
        sleep(5)
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
        print("Got here")
        print(self.line_1_distance)
        print(self.line_2_distance)
        if (not abs(self.line_1_distance-self.line_2_distance) < 0.1):
            if (self.line_1_distance > self.line_2_distance):
                self.turn_right(msg)
                print("Right")
            else:
                self.turn_left(msg)
                print("Left")
            print("Adjusting")
        else:
            self.move_forward(msg)
            print("Moving Forward")

        sleep(0.5)

def main():
    rclpy.init()
    node = WallFollowerNode()
    rclpy.spin(node) #infinite loop
    rclpy.shutdown() # when is spin complete


if __name__ == '__main__':
    main()
