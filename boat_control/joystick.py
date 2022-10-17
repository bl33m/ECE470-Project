
import math
import numpy as np
import rclpy
from rclpy.node import Node
#from rclpy.clock import Clock

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Joy

from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry


class JoyCtl(Node):
    def __init__(self):
        self.left_thrust = 0
        self.right_thrust = 0

        self.joystick_sub = self.create_subscription(Joy, 'joy', self.read_joy, 10)

        self.left_thrust_pub = self.create_publisher(Float64, 'usv/left/thruster/cmd_thrust', 10)
        self.right_thrust_pub = self.create_publisher(Float64, 'usv/right/thruster/cmd_thrust', 10)
        self.timer = self.create_timer(0.1, self.publish_thrust)
    
    def read_joy(msg):
        self.left_thrust = msg.axes[1]*100
        self.right_thrust = msg.axes[4]*100

    def publish_thrust(self):
        self.left_thrust_pub.publish(self.left_thrust)
        self.right_thrust_pub.publish(self.right_thrust)

def main():
    rclpy.init()

    joyctl = JoyCtl()

    rclpy.spin(joyctl)
    joyctl.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
