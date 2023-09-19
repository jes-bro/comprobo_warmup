import rclpy
from rclpy.node import Node # Import super/base class required to create ROS Node
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker

class CreateSphereNode(Node):
    def __init__(self):
        super().__init__('send_message')
        # Args: interval between invocations of the timer (period), (callback)
        self.create_timer(0.1, self.run_loop) # executes run loop 10x a second - how does callback work exactly
                # create publisher to publish messages to topic
        self.pub = self.create_publisher(Marker, 'marker', 10)

    def run_loop(self):
        msg = Marker()
        msg.pose.position.x = 1.0
        msg.pose.position.y = 2.0
        msg.header.frame_id = 'base_link'
        msg.type = Marker.SPHERE
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        msg.scale.x = 1.0
        msg.scale.y = 0.1
        msg.scale.z = 0.1
        msg.color.a = 1.0; # Don't forget to set the alpha!
        msg.color.r = 0.0
        msg.color.g = 1.0
        msg.color.b = 0.0
        msg.header.stamp = self.get_clock().now().to_msg() 
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CreateSphereNode()
    rclpy.spin(node) #infinite loop
    rclpy.shutdown() # when is spin complete


if __name__ == '__main__':
    main()
