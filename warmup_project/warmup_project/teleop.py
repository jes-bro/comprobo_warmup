import rclpy
from rclpy.node import Node  # Import super/base class required to create ROS Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
import tty
import select
import sys
import termios


class TeleopNode(Node):
    """
    A node used for teleoperation purposes.
    It extends the rclpy's Node class and integrates functionalities to get keyboard inputs
    and to publish messages to control robot's movement based on the input.
    """
    def __init__(self):
        super().__init__("teleop_node")
        # Args: interval between invocations of the timer (period), (callback)
        self.create_timer(
            0.1, self.run_loop
        )  # executes run loop 10x a second - how does callback work exactly
        # create publisher to publish messages to topic
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

    def getKey(self):
        """
        Gets a single key input from the user without requiring to hit enter.
        It makes use of termios and tty modules to get the key input in a raw mode,
        and then restores the previous settings to avoid affecting the terminal behavior.

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

    def run_loop(self):
        """
        A loop that runs at a specified interval defined by the timer in the __init__ method.
        It gets a key input and publishes a message with linear and angular velocities
        based on the key pressed to control the robot movement. If Ctrl+C is pressed, it
        destroys the node and exits.
        """
        key = self.getKey()
        msg = Twist()
        if key == "\x03":
            self.destroy_node()

        match key:
            # Move forward
            case "w":
                msg.linear.x = 0.2
                msg.angular.z = 0.0
            # Turn left
            case "a":
                msg.linear.x = 0.0
                msg.angular.z = 0.5
            # Move backward
            case "s":
                msg.linear.x = -0.2
                msg.angular.z = 0.0
            # Turn right
            case "d":
                msg.linear.x = 0.0
                msg.angular.z = -0.5

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TeleopNode()
    rclpy.spin(node)  # infinite loop
    rclpy.shutdown()  # when is spin complete


if __name__ == "__main__":
    main()
