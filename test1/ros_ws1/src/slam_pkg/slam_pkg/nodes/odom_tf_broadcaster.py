#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        
        # Initialize TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscribe to odometry topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',  # Your odometry topic from Gazebo bridge
            self.odom_callback,
            10
        )
        
        self.get_logger().info('Odom TF Broadcaster started. Listening to /odom')

    def odom_callback(self, msg):
        # Create a transform message
        t = TransformStamped()
        
        # Set header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'  # Parent frame
        t.child_frame_id = 'base_link'  # Child frame
        
        # Set translation from odometry message
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        # Set rotation from odometry message
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()