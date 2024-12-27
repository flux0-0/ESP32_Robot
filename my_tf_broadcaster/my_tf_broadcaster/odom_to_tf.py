import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Quaternion, TransformStamped
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Subscription to encoder data
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'encoder_data',
            self.encoder_callback,
            10)
        
        # Publisher for the Odometry message
        self.publisher = self.create_publisher(Odometry, 'odom', 10)

        # Creating a Transform Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Robot parameters
        self.wheel_diameter = 0.065
        self.wheel_base = 0.185
        self.encoder_resolution = 508.1
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.previous_left_encoder = 0
        self.previous_right_encoder = 0

        # Create a Timer to broadcast odometry and tf at 10Hz (0.1s interval)
        self.timer = self.create_timer(0.125, self.publish_odometry_and_tf)  # 10Hz

    def encoder_callback(self, msg):
        left_encoder = msg.data[0]
        right_encoder = msg.data[1]

        delta_left = left_encoder - self.previous_left_encoder
        delta_right = right_encoder - self.previous_right_encoder

        self.previous_left_encoder = left_encoder
        self.previous_right_encoder = right_encoder

        # Calculate distance moved by each wheel
        left_distance = (delta_left * (math.pi * self.wheel_diameter)) / self.encoder_resolution
        right_distance = (delta_right * (math.pi * self.wheel_diameter)) / self.encoder_resolution
        distance = (left_distance + right_distance) / 2.0
        angle_change = (right_distance - left_distance) / self.wheel_base

        # Update odometry
        self.x += distance * math.cos(self.theta + angle_change / 2.0)
        self.y += distance * math.sin(self.theta + angle_change / 2.0)
        self.theta += angle_change

    def create_quaternion(self, yaw):
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0)
        )

    def publish_odometry_and_tf(self):
        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = self.create_quaternion(self.theta)
        
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(odom_msg)

        # Broadcast transform
        self.broadcast_transform(odom_msg)

    def broadcast_transform(self, odom_msg):
        # Create a TransformStamped message to broadcast the TF
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = odom_msg.pose.pose.position.x
        transform.transform.translation.y = odom_msg.pose.pose.position.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation = odom_msg.pose.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

