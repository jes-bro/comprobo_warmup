import rclpy
from rclpy.node import Node # Import super/base class required to create ROS Node
# Topics are tight so they expect a type of data to go over a topic
# Topic is channel 
# Will stuff break if you use different msg types
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
import tty
import select
import sys
import termios
from time import sleep

class DriveSquareNode(Node):
    def __init__(self):
        super().__init__('square_node')
        # Args: interval between invocations of the timer (period), (callback)
        self.create_timer(0.1, self.run_loop) # executes run loop 10x a second - how does callback work exactly
        # create publisher to publish messages to topic
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def getKey(self):
        settings = termios.tcgetattr(sys.stdin)
        key = None
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    
    def move_forward(self, msg):
        print("Got to move fwd")
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        self.pub.publish(msg)
        sleep(5)
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def turn_left(self, msg):
        print("Got to turn left")
        msg.linear.x = 0.2
        msg.angular.z = 0.1
        self.pub.publish(msg)
        sleep(3)
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def run_loop(self):
        msg = Twist()
        self.move_forward(msg)
        self.turn_left(msg)

def main():
    rclpy.init()
    node = DriveSquareNode()
    rclpy.spin(node) #infinite loop
    rclpy.shutdown() # when is spin complete


if __name__ == '__main__':
    main()
