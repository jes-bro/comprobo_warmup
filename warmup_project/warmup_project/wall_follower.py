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
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos, radians
from time import sleep

class WallFollowerNode(Node):

    def __init__(self):
        super().__init__('wall_follower_node')
        # Args: interval between invocations of the timer (period), (callback)
        self.create_timer(0.1, self.run_loop) # executes run loop 10x a second - how does callback work exactly
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        # create publisher to publish messages to topic
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.line_1_distance = 0
        self.line_2_distance = 1

    def process_scan(self, laser_data):
        self.ranges = np.array(list(laser_data.ranges))
        print(f"ranges: {self.ranges}")
        self.line_1_distance = self.ranges[135]
        self.line_2_distance = self.ranges[45]
        print("-------------------------")
        points = self.polar_to_cartesian()
        accumulator = self.generate_hough_space(points)
        self.generate_heat_map(accumulator)

    def polar_to_cartesian(self):
        points = []
        for angle in range(len(self.ranges)):
            point = (self.ranges[angle]*cos(radians(angle)), self.ranges[angle]*sin(radians(angle)))
            points.append(point)
        return points

    def generate_hough_space(self, points):

        thetas = np.linspace(0, np.pi, 180)
        r_min, r_max, r_step = -5, 5, 0.025
        r_values = np.arange(r_min, r_max + r_step, r_step)
        
        # Initialize matrix with zeros
        accumulator = np.zeros((len(r_values), len(thetas)))
        
        for point in points:
            for index, theta in enumerate(thetas):
                r = point[0] * cos(theta) + point[1] * sin(theta)
                
                # Find the corresponding r bucket
                r_bucket = int((r - r_min) / r_step)
                
                if 0 <= r_bucket < len(r_values):
                    accumulator[r_bucket][index] += 1
                    
        return accumulator
    
    def generate_heat_map(self, accumulator):
        # Generate a figure and axis
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # Create heatmap
        sns.heatmap(accumulator, cmap='viridis', ax=ax)

        # Define x and y labels
        ax.set_xlabel('Theta (radians)')
        ax.set_ylabel('$\\rho$ (meters)')
        ax.set_title("Hough Space: $\\rho - \\theta$ Parameterization")
        
        # Set y-limits
        r_min, r_max, r_step = -5, 5, 0.025
        ax.set_ylim(len(accumulator) - 1, 0)  # Reverse the limits as heatmaps usually plot top-down
        
        # Adjust y-ticks
        r_values = np.arange(r_min, r_max + r_step, r_step)
        y_ticks = np.arange(0, len(r_values), 10)  # Show every 5th tick for clarity, adjust as needed
        ax.set_yticks(y_ticks)
        ax.set_yticklabels([round(r_values[i], 2) for i in y_ticks])
        
        plt.show()
    
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
