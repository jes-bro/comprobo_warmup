import rclpy
from rclpy.node import Node  # Import super/base class required to create ROS Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
import tty
import select
import sys
import termios
from time import sleep


class DriveSquareNode(Node):
    """
    A DriveSquareNode is a ROS (Robot Operating System) node used for driving a robot
    in a square path. It inherits from the rclpy's Node class and integrates functionalities
    to control the robot's linear and angular velocities at fixed intervals to drive in a square.
    """

    def __init__(self):
        super().__init__("square_node")
        self.create_timer(0.1, self.run_loop)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

    def getKey(self):
        """
        Gets a single key input from the user without requiring to hit enter.
        It employs termios and tty modules to obtain the key input in a raw mode,
        then restores the initial settings to maintain the terminal behavior intact.

        Returns:
            str: The key that was pressed.
        """
        settings = termios.tcgetattr(sys.stdin)
        key = None
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def move_forward(self, msg):
        """
        Controls the robot to move forward by setting appropriate linear and angular
        velocities in the Twist message and publishing it. The robot moves forward for
        a fixed duration before stopping.

        Args:
            msg (Twist): The Twist message where the velocity commands are set.
        """
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        self.pub.publish(msg)
        sleep(5)
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def turn_left(self, msg):
        """
        Controls the robot to turn left by setting the appropriate linear and angular
        velocities in the Twist message and publishing it. The robot turns for a fixed
        duration before stopping.

        Args:
            msg (Twist): The Twist message where the velocity commands are set.
        """
        msg.linear.x = 0.0
        msg.angular.z = 0.5
        self.pub.publish(msg)
        sleep(3.3)
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def run_loop(self):
        """
        The main loop that is periodically invoked by the timer created in the __init__ method.
        It calls the move_forward and turn_left methods in sequence to drive the robot in a
        square path.
        """
        msg = Twist()
        self.move_forward(msg)
        self.turn_left(msg)


def main():
    rclpy.init()
    node = DriveSquareNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
