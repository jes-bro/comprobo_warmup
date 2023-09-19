import rclpy
from rclpy.node import Node  # Import super/base class required to create ROS Node

# Topics are tight so they expect a type of data to go over a topic
# Topic is channel
# Will stuff break if you use different msg types
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import tty
import select
import sys
import termios
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos, radians
from time import sleep


class WallFollowerNode(Node):
    """
    A ROS2 node designed to govern the wall-following behavior of a robot.

    This node subscribes to laser scan messages to receive data about its surroundings and
    uses that data to perform navigation tasks. It employs a variety of geometric and visualization
    techniques, including the Hough Transform to identify lines in the laser scan data and seaborn
    to visualize the Hough space as a heat map.

    The node publishes Twist messages to control the robot's movements, and visualization Marker messages
    to visualize the identified lines from the Hough Transform in a visualization tool such as RViz.
    The node performs a wall-following behavior, attempting to maintain a consistent distance from
    the walls it is following.
    """

    def __init__(self):
        super().__init__("wall_follower_node")
        # Args: interval between invocations of the timer (period), (callback)
        self.create_timer(0.1, self.run_loop)  # executes run loop 10x a second
        self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        # create publisher to publish messages to topic
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub2 = self.create_publisher(Marker, "visualization_marker", 10)
        self.line_1_distance = 0
        self.line_2_distance = 1
        self.thetas = np.linspace(0, np.pi, 180)
        self.r_min, self.r_max, self.r_step = -5, 5, 0.025
        self.r_values = np.arange(self.r_min, self.r_max + self.r_step, self.r_step)

    def process_scan(self, laser_data):
        """Process laser data to get the distances and convert the polar coordinates to Cartesian for Hough Transform.

        Args:
            laser_data: The data from the laser scan sensor.
        """
        self.ranges = np.array(list(laser_data.ranges))
        self.line_1_distance = self.ranges[135]
        self.line_2_distance = self.ranges[45]
        points = self.polar_to_cartesian()
        accumulator = self.generate_hough_space(points)
        self.plot_lines(accumulator)
        # self.generate_heat_map(accumulator)

    def polar_to_cartesian(self):
        """Convert the polar coordinates from the laser scan to Cartesian coordinates.

        Returns:
            list: A list of Cartesian coordinates.
        """
        points = []
        for angle in range(len(self.ranges)):
            point = (
                self.ranges[angle] * cos(radians(angle)),
                self.ranges[angle] * sin(radians(angle)),
            )
            points.append(point)
        return points

    def generate_hough_space(self, points):
        """Generate the Hough space accumulator array from the Cartesian coordinates.

        Args:
            points (list): A list of Cartesian coordinates.

        Returns:
            numpy.ndarray: The Hough space accumulator array.
        """
        # Initialize matrix with zeros
        accumulator = np.zeros((len(self.r_values), len(self.thetas)))

        for point in points:
            for index, theta in enumerate(self.thetas):
                r = point[0] * cos(theta) + point[1] * sin(theta)

                # Find the corresponding r bucket
                r_bucket = int((r - self.r_min) / self.r_step)

                if 0 <= r_bucket < len(self.r_values):
                    accumulator[r_bucket][index] += 1

        return accumulator

    def plot_lines(self, accumulator):
        """Identify and plot lines in the laser scan data using the Hough Transform.

        Args:
            accumulator (numpy.ndarray): The Hough space accumulator array.
        """
        max_value = np.max(accumulator)

        # Set the threshold as 70% of the max value
        threshold = 0.7 * max_value

        # Extract the coordinates above the threshold
        hot_spots = np.where(accumulator >= threshold)

        for y, x in zip(*hot_spots):
            rho = self.r_values[y]
            theta = self.thetas[x]

            # Convert (rho, theta) to Cartesian coordinates (start and end points of the line)
            x1 = rho * np.cos(theta) - 500 * np.sin(theta)
            y1 = rho * np.sin(theta) + 500 * np.cos(theta)
            x2 = rho * np.cos(theta) + 500 * np.sin(theta)
            y2 = rho * np.sin(theta) - 500 * np.cos(theta)

            start_point = Point()
            start_point.x = x1
            start_point.y = y1
            start_point.z = 0.0
            end_point = Point()
            end_point.x = x2
            end_point.y = y2
            end_point.z = 0.0

            # Publish the line using publish_line function
            self.publish_line(start_point, end_point)

    def publish_line(self, start_point, end_point):
        """Publish a line as a Marker message between two points.

        Args:
            start_point (geometry_msgs.msg.Point): The start point of the line.
            end_point (geometry_msgs.msg.Point): The end point of the line.
        """
        marker = Marker()
        marker.header.frame_id = "/odom"
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02  # Width of the line
        marker.color.r = 1.0
        marker.color.a = 1.0
        marker.points.append(start_point)
        marker.points.append(end_point)

        self.pub2.publish(marker)

    def generate_heat_map(self, accumulator):
        """Generate a heat map visualization of the Hough space.

        Args:
            accumulator (numpy.ndarray): The Hough space accumulator array.
        """
        # Generate a figure and axis
        fig, ax = plt.subplots(figsize=(10, 10))

        # Create heatmap
        sns.heatmap(accumulator, cmap="viridis", ax=ax)

        # Define x and y labels
        ax.set_xlabel("Theta (radians)")
        ax.set_ylabel("$\\rho$ (meters)")
        ax.set_title("Hough Space: $\\rho - \\theta$ Parameterization")

        # Set y-limits
        r_min, r_max, r_step = -5, 5, 0.025
        ax.set_ylim(
            len(accumulator) - 1, 0
        )  # Reverse the limits as heatmaps usually plot top-down

        # Adjust y-ticks
        r_values = np.arange(r_min, r_max + r_step, r_step)
        y_ticks = np.arange(0, len(r_values), 10)
        ax.set_yticks(y_ticks)
        ax.set_yticklabels([round(r_values[i], 2) for i in y_ticks])

        plt.show()

    def move_forward(self, msg):
        """Move the robot forward a set distance.

        Args:
            msg (geometry_msgs.msg.Twist): The Twist message controlling the robot's movement.
        """
        msg.linear.x = 0.05
        msg.angular.z = 0.0
        self.pub.publish(msg)
        sleep(5)
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def turn_left(self, msg):
        """Rotate the robot to the left a set angle.

        Args:
            msg (geometry_msgs.msg.Twist): The Twist message controlling the robot's movement.
        """
        msg.angular.z = 0.5
        self.pub.publish(msg)
        sleep(0.1)
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def turn_right(self, msg):
        """Rotate the robot to the right a set angle.

        Args:
            msg (geometry_msgs.msg.Twist): The Twist message controlling the robot's movement.
        """
        msg.angular.z = -0.5
        self.pub.publish(msg)
        sleep(0.1)
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def run_loop(self):
        msg = Twist()
        if not abs(self.line_1_distance - self.line_2_distance) < 0.1:
            if self.line_1_distance > self.line_2_distance:
                self.turn_right(msg)
            else:
                self.turn_left(msg)
        else:
            self.move_forward(msg)

        sleep(0.5)


def main():
    rclpy.init()
    node = WallFollowerNode()
    rclpy.spin(node)  # infinite loop
    rclpy.shutdown()  # when is spin complete


if __name__ == "__main__":
    main()
