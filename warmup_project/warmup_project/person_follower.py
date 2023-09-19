import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import tty
import select
import sys
import termios
from time import sleep
import math


class PersonFollowerNode(Node):
    """
    ROS Node to follow a person based on LaserScan data. The node uses a simple algorithm to determine the direction
    and distance to the closest point in a subset of the laser scan range and moves towards that point at a specified follow distance.
    """
    def __init__(self):
        super().__init__("person_follower_node")
        self.create_timer(
            0.1, self.run_loop
        )  # Timer to call run_loop at 0.1s intervals
        self.create_subscription(
            LaserScan, "scan", self.process_scan, 10
        )  # Subscribes to LaserScan messages on the "scan" topic
        self.pub = self.create_publisher(
            Twist, "cmd_vel", 10
        )  # Publisher to send Twist messages on "cmd_vel" topic
        self.ranges = []  # List to store processed LaserScan ranges
        self.max_scan_distance = 5  # Maximum distance to consider for scans
        self.follow_distance = 1  # Desired following distance

    def process_scan(self, laser_data):
        """
        Callback method to process LaserScan data. It extracts a subset of the ranges from the front of the robot,
        filters out ranges beyond a maximum distance and stores the processed ranges in self.ranges.

        Args:
            laser_data (LaserScan): The LaserScan message containing the scan data.
        """
        ranges = laser_data.ranges  # LaserScan ranges
        # Extracting left, right, and front ranges with necessary transformations
        ranges_left = ranges[0:45]
        ranges_right = ranges[314:359]
        ranges_left_reversed = list(reversed(ranges_left))
        ranges_right_reversed = list(reversed(ranges_right))
        ranges_front = ranges_left_reversed + ranges_right_reversed

        # Filtering out ranges beyond the max_scan_distance
        self.ranges = [
            math.inf if x > self.max_scan_distance else x for x in ranges_front
        ]

    def move_forward(self, msg):
        """Controls the robot to move forward for a short duration."""
        msg.linear.x = 0.05
        self.pub.publish(msg)
        sleep(0.1)
        msg.linear.x = 0.0
        self.pub.publish(msg)

    def move_backward(self, msg):
        """Controls the robot to move backward for a short duration."""
        msg.linear.x = -0.05
        self.pub.publish(msg)
        sleep(0.1)
        msg.linear.x = 0.0
        self.pub.publish(msg)

    def turn_left(self, msg):
        """Controls the robot to turn left for a short duration."""
        msg.angular.z = 0.1
        self.pub.publish(msg)
        sleep(0.1)
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def turn_right(self, msg):
        """Controls the robot to turn right for a short duration."""
        msg.angular.z = -0.1
        self.pub.publish(msg)
        sleep(0.1)
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def run_loop(self):
        """
        Main loop called periodically to control the robot's motion based on the processed LaserScan data.
        It first rotates to face the closest point and then moves forward or backward to maintain a desired following distance.
        """
        msg = Twist()
        # Handle case of no valid LIDAR data
        if not self.ranges:
            print("Empty!")
            return

        # Find the closest point and its index in the scan data
        closest_point = min(self.ranges)
        closest_point_idx = self.ranges.index(closest_point)

        # Calculate the error in the robot's angular position
        angular_error = closest_point_idx - 45

        # Correct the angular position if error is beyond a threshold
        if not (-5 < angular_error < 5):
            self.turn_right(msg) if angular_error > 0 else self.turn_left(msg)
            return

        # Correct the linear position if error is beyond a threshold
        distance_error = self.follow_distance - closest_point
        if not abs(distance_error) < 0.2:
            self.move_forward(msg) if distance_error < 0 else self.move_backward(msg)
            return

        sleep(0.5)


def main():
    rclpy.init()
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
