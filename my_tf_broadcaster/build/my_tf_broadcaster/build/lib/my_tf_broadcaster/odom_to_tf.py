import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        
        # Subscribing to the /odom topic
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # Creating a Transform Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def odom_callback(self, msg):
        # Create a TransformStamped message to broadcast the TF
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp  # Use the timestamp from the Odometry message
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        # Use the position from the Odometry message
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = 0.0

        # Use the orientation from the Odometry message
        transform.transform.rotation = msg.pose.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

