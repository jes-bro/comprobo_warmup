import rclpy
from rclpy.node import Node  # Import super/base class required to create ROS Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import tty
import select
import sys
import termios
from time import sleep
import math


class FiniteStateControllerNode(Node):
    """
    A ROS node that implements a finite state controller for a robot. The robot can either follow a person
    based on LiDAR sensor data or drive in a square path if no person is detected.

    The node subscribes to LaserScan messages to get data from a LiDAR sensor and publishes Twist messages
    to control the robot's movement.
    """

    def __init__(self):
        super().__init__("finite_state_controller_node")
        # Args: interval between invocations of the timer (period), (callback)
        self.create_timer(
            0.1, self.run_loop
        )  # executes run loop 10x a second - how does callback work exactly
        self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        # create publisher to publish messages to topic
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.ranges = []
        self.max_scan_distance = 5
        self.follow_distance = 1
        self.state = "drive_square"

    def process_scan(self, laser_data):
        """
        Callback function to process LaserScan data.

        Extracts and stores range data from the front 90 degrees field of view of the robot, filtering out
        measurements beyond max_scan_distance.

        Args:
            laser_data (LaserScan): The incoming laser data message.
        """
        ranges = laser_data.ranges
        ranges_list = list(ranges)

        ranges_left = ranges_list[0:45]
        ranges_right = ranges_list[314:359]

        ranges_left_reversed = []
        ranges_right_reversed = []

        for i in reversed(range(0, 45)):
            ranges_left_reversed.append(ranges_left[i])
            ranges_right_reversed.append(ranges_right[i])

        ranges_front = ranges_left_reversed + ranges_right_reversed
        self.ranges = [
            math.inf if x > self.max_scan_distance else x for x in ranges_front
        ]

    def move_forward(self, msg):
        """
        Moves the robot forward by publishing a Twist message with a positive linear velocity.

        Args:
            msg (Twist): The Twist message object to publish the velocity commands.
        """
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
        msg.angular.z = 0.1
        self.pub.publish(msg)
        sleep(0.1)
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def turn_right(self, msg):
        msg.angular.z = -0.1
        self.pub.publish(msg)
        sleep(0.1)
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def move_forward_square(self, msg):
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        self.pub.publish(msg)
        sleep(5)
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def turn_left_square(self, msg):
        msg.linear.x = 0.0
        msg.angular.z = 0.5
        self.pub.publish(msg)
        sleep(3.3)
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def run_loop(self):
        msg = Twist()
        print("loop")
        # Check to see if LIDAR data empty
        if len(self.ranges) == 0:
            print("Empty!")
            return

        is_no_detection = all(math.isinf(x) for x in self.ranges)

        match self.state:
            case "drive_square":
                if is_no_detection:
                    self.move_forward_square(msg)
                    self.turn_left_square(msg)
                    return
                else:
                    self.state = "follow_person"
                    return
            case "follow_person":
                if not is_no_detection:
                    closest_point = min(self.ranges)

                    closest_point_idx = self.ranges.index(closest_point)

                    # Index corresponds to point's angle relative to NEATO
                    angular_error = closest_point_idx - 45

                    # Adjust angular position (rotate)
                    if not (-5 < angular_error < 5):
                        if angular_error > 0:
                            self.turn_right(msg)
                        else:
                            self.turn_left(msg)
                        return

                    # Adjust linear position (translate)
                    if not abs(self.follow_distance - closest_point) < 0.1:
                        if self.follow_distance - closest_point < 0:
                            self.move_forward(msg)
                        else:
                            self.move_backward(msg)
                        return
                else:
                    self.state = "drive_square"
                    return

        sleep(0.5)


def main():
    rclpy.init()
    node = FiniteStateControllerNode()
    rclpy.spin(node)  # infinite loop
    rclpy.shutdown()  # when is spin complete


if __name__ == "__main__":
    main()
