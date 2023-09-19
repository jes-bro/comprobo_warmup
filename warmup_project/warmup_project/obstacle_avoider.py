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


class ObstacleAvoiderNode(Node):
    """
    ROS node for an obstacle avoidance system that utilizes a LiDAR sensor to identify obstacles and navigate around them.

    This node subscribes to LaserScan messages, processes the laser data to identify the closest obstacles, and decides
    whether to move forward, backward, turn left, or turn right to avoid the obstacle. It publishes Twist messages to control
    the robot's movements based on the processed laser data.

    Attributes:
        pub (Publisher): Publisher to send Twist messages to control the robot's movement.
        ranges (list): List to store the laser range data.
        distance_threshold (float): The distance threshold for considering an object as an obstacle.
        turning_right (bool): Flag to indicate the current turning direction of the robot.
    """

    def __init__(self):
        super().__init__("obstacle_avoider_node")
        # Args: interval between invocations of the timer (period), (callback)
        self.create_timer(
            0.1, self.run_loop
        )  # executes run loop 10x a second - how does callback work exactly
        self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        # create publisher to publish messages to topic
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.ranges = []
        self.distance_threshold = 1
        self.turning_right = True

    def process_scan(self, laser_data):
        """Process LaserScan data to extract the necessary range information for obstacle avoidance.

        Args:
            laser_data (LaserScan): The LaserScan data containing range information.
        """
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
        """Move the robot forward by publishing a Twist message with the specified linear velocity.

        Args:
            msg (Twist): The Twist message to set the linear and angular velocities for robot movement.
        """
        msg.linear.x = 0.05
        msg.angular.z = 0.0
        self.pub.publish(msg)
        sleep(0.3)
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def move_backward(self, msg):
        """Move the robot backward by publishing a Twist message with the specified linear velocity.

        Args:
            msg (Twist): The Twist message to set the linear and angular velocities for robot movement.
        """
        msg.linear.x = -0.05
        msg.angular.z = 0.0
        self.pub.publish(msg)
        sleep(0.1)
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def turn_left(self, msg):
        """Turn the robot to the left by publishing a Twist message with the specified angular velocity.

        Args:
            msg (Twist): The Twist message to set the linear and angular velocities for robot movement.
        """
        self.turning_right = False
        msg.angular.z = 0.1
        self.pub.publish(msg)
        sleep(0.2)
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def turn_right(self, msg):
        """Turn the robot to the right by publishing a Twist message with the specified angular velocity.

        Args:
            msg (Twist): The Twist message to set the linear and angular velocities for robot movement.
        """
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
        if len(self.ranges) == 0:
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
            if angular_error > 0:
                self.turn_left(msg)
            else:
                self.turn_right(msg)

        self.move_forward(msg)
        sleep(0.5)


def main():
    rclpy.init()
    node = ObstacleAvoiderNode()
    rclpy.spin(node)  # infinite loop
    rclpy.shutdown()  # when is spin complete


if __name__ == "__main__":
    main()
