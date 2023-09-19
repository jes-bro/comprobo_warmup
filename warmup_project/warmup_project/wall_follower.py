import rclpy
from rclpy.node import Node  # Import the Node class from the rclpy.node module
from geometry_msgs.msg import Twist  # Importing necessary message types
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import tty
import select
import sys
import termios
from time import sleep
import math


class WallFollowerNode(Node):
    """ROS node that follows walls at a safe distance using sensor data."""

    def __init__(self):
        """Initialize the wall follower node with necessary subscriptions and publishers."""
        super().__init__('wall_follower_node')

        # Creating a timer to call the run_loop method at regular intervals
        self.create_timer(0.1, self.run_loop) 

        # Creating a subscription to get data from the 'scan' topic
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)

        # Creating a publisher to send messages to the 'cmd_vel' topic
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initializing distances to keep track of distance to lines detected
        self.line_1_distance = 0
        self.line_2_distance = 1

    def process_scan(self, laser_data):
        """Process laser scan data to update line distances."""
        ranges = laser_data.ranges
        self.line_1_distance = ranges[135]
        self.line_2_distance = ranges[45]

    def move_forward(self, msg):
        """Move the robot forward for a set duration before stopping."""
        msg.linear.x = 0.05
        msg.angular.z = 0.0
        self.pub.publish(msg)
        sleep(5)  
        msg.linear.x = 0.0
        self.pub.publish(msg)  

    def turn_left(self, msg):
        """Rotate the robot left for a short duration before stopping."""
        msg.angular.z = 0.5
        self.pub.publish(msg)
        sleep(0.1) 
        msg.angular.z = 0.0  
        self.pub.publish(msg) 

    def turn_right(self, msg):
        """Rotate the robot right for a short duration before stopping."""
        msg.angular.z = -0.5 
        self.pub.publish(msg)  
        sleep(0.1)
        msg.angular.z = 0.0  
        self.pub.publish(msg)

    def run_loop(self):
        """Main loop where the robot decides to turn or move forward based on sensor data."""
        msg = Twist()  # Creating a new Twist message

        # Decide whether to turn or move forward based on the line distances
        if not abs(self.line_1_distance - self.line_2_distance) < 0.1:
            if self.line_1_distance > self.line_2_distance:
                self.turn_right(msg)
            else:
                self.turn_left(msg)
        else:
            self.move_forward(msg)

        sleep(0.5)  # Pause before the next loop iteration

def main():
    """Main function to initialize ROS and spin the node."""
    rclpy.init()  # Initialize ROS
    node = WallFollowerNode()  # Create a new WallFollowerNode instance
    rclpy.spin(node)  # Spin the node to keep it running
    rclpy.shutdown()  # Shutdown ROS when the node stops spinning

if __name__ == '__main__':
    main()  # Run the main function when the script is executed
