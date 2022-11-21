import math
import numpy as np
import rclpy
from rclpy.node import Node
#from rclpy.clock import Clock

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Joy, Imu

from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry

from boat_control.utils import quaternion_from_euler

class OdometryNode(Node):
    # Initialize some variables
    
    gyro_yaw = 0.0
    gyro_pitch = 0.0
    gyro_roll = 0.0
    
    left_speed = 0.0
    right_speed = 0.0

    Accelx = 0.0
    Accely = 0.0
    Accelz = 0.0


    x = 0.0 # x robot's position
    y = 0.0 # y robot's position
    theta = 0.0 # heading angle
    l_wheels = 2.696 # Distance between right and left propelers

    last_time = 0.0
    current_time = 0.0

    def __init__(self):
        super().__init__('minimal_subscriber')
       
        self.subscription_Imu = self.create_subscription(Imu, '/usv/imu/data', self.callback_Imu, 10)
        self.subscription_left_thrust = self.create_subscription(Float64,  '/usv/left/thrust/cmd_thrust', self.callback_left, 10)
        self.subscription_right_thrust = self.create_subscription(Float64, '/usv/right/thrust/cmd_thrust', self.callback_right, 10)

        self.last_time = self.get_clock().now().nanoseconds/1e9
        
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10) 
        #self.timer = self.create_timer(0.1, self.timer_callback_odom) #It creates a timer to periodically publish the odometry.
        
        self.tf_broadcaster = TransformBroadcaster(self) # To broadcast the transformation between coordinate frames.
    
    def callback_Imu(self, msg):
        self.gyro_yaw = msg.angular_velocity.z
        print(self.gyro_yaw)
    
    def callback_left(self, msg):
        self.left_speed = msg.data
    def callback_right(self, msg):
        self.right_speed = msg.data
    #def callback_Gp(self, msg):
    #    self.gyro_pitch = msg.data
    #def callback_Gr(self, msg):
    #    self.gyro_roll = msg.data
    

    def timer_callback_odom(self):

        self.current_time = self.get_clock().now().nanoseconds/1e9
        dt = self.current_time - self.last_time # DeltaT
        
        vl = self.left_speed
        vr = self.right_speed
        
        v = (vl+vr)/2 # ... Linear velocity of the robot
        w = (vl-vr)/self.l_wheels # ... Angular velocity of the robot
        self.x += v*np.cos(self.theta)*dt # ...Position
        self.y += v*np.sin(self.theta)*dt # ...Position
        self.theta += self.gyro_yaw*dt # ...Heading angle

        position = [self.x, self.y, 0.0]
        quater = quaternion_from_euler(0.0, 0.0, self.theta)
        print("position: ", position)
        print("orientation: ", quater)
        print("gyro_yaw ", self.gyro_yaw)


        # We need to set an odometry message and publish the transformation between two coordinate frames
        # Further info about odometry message: https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html
        # Further info about tf2: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html
        # Further info about coordinate frames in ROS: https://www.ros.org/reps/rep-0105.html

        frame_id = 'odom'
        child_frame_id = 'usv/base_link'
        
        self.broadcast_tf(position, quater, frame_id, child_frame_id)  # Before creating the odometry message, go to the broadcast_tf function and complete it.
        
        odom = Odometry()
        odom.header.frame_id = frame_id
        odom.header.stamp = self.get_clock().now().to_msg()


        odom.pose.pose.position.x = self.x # ...
        odom.pose.pose.position.y = self.y # ...
        odom.pose.pose.position.z = 0.0 # ... 
        odom.pose.pose.orientation.x = quater[0]
        odom.pose.pose.orientation.y = quater[1] # ...
        odom.pose.pose.orientation.z = quater[2] # ...
        odom.pose.pose.orientation.w = quater[3] # ...

        odom.child_frame_id = child_frame_id
        odom.twist.twist.linear.x = v*np.cos(self.theta) # ...
        odom.twist.twist.linear.y = v*np.sin(self.theta) # ...
        odom.twist.twist.linear.z = 0.0 # ...
        odom.twist.twist.angular.x = 0.0 # ...
        odom.twist.twist.angular.y = 0.0 # ...
        odom.twist.twist.angular.z = self.gyro_yaw # ...

        self.odom_pub.publish(odom)

        self.last_time = self.current_time
        
    def broadcast_tf(self, pos, quater, frame_id, child_frame_id):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id

        t.transform.translation.x = pos[0] # ...
        t.transform.translation.y = pos[1] # ...
        t.transform.translation.z = pos[2] # ...

        t.transform.rotation.x = quater[0] # ...
        t.transform.rotation.y = quater[1] # ...
        t.transform.rotation.z = quater[2] # ...
        t.transform.rotation.w = quater[3] # ...

        self.tf_broadcaster.sendTransform(t)
    
    
def main(args=None):
    rclpy.init(args=args)

    odom_node = OdometryNode()

    rclpy.spin(odom_node)
    odom_node.file_object_results.close()
    odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
