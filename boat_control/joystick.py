
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
        super().__init__('minimal_subscriber')
   
        self.left_thrust = 0.0
        self.right_thrust = 0.0

        self.elbow = 0.0
        self.right_finger = 0.0
        self.left_finger = 0.0

        self.joystick_sub = self.create_subscription(Joy, '/joy', self.read_joy, 10)
                            
        self.left_thrust_pub = self.create_publisher(Float64, '/usv/left/thrust/cmd_thrust', 10)
        self.right_thrust_pub = self.create_publisher(Float64, '/usv/right/thrust/cmd_thrust', 10)
        self.left_finger_pub = self.create_publisher(Float64, '/usv/arm/gripper/joint/finger_left/cmd_pos', 10)
        self.right_finger_pub = self.create_publisher(Float64, '/usv/arm/gripper/joint/finger_right/cmd_pos', 10)
        self.timer = self.create_timer(0.1, self.pub_joy_ctl)
    
    def read_joy(self, msg):
        self.left_thrust = msg.axes[1]*100
        self.right_thrust = msg.axes[4]*100
        self.right_finger = msg.buttons[0]*1.0
        self.left_finger = msg.buttons[0]*1.0

    def pub_joy_ctl(self):
        right_thrust = Float64()
        left_thrust = Float64()
        left_finger = Float64()
        right_finger = Float64()
        
        right_thrust.data = self.right_thrust
        left_thrust.data = self.left_thrust
        right_finger.data = self.right_finger
        left_finger.data = self.left_finger
        
        self.left_thrust_pub.publish(left_thrust)
        self.right_thrust_pub.publish(right_thrust)
        self.right_finger_pub.publish(right_finger)
        self.left_finger_pub.publish(left_finger)

def main(args=None):
    rclpy.init(args=args)

    joyctl = JoyCtl()

    rclpy.spin(joyctl)
    joyctl.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
