#!/usr/bin/env python3
"""
Pose Bridge Node

Subscribes to /poses (NamedPoseArray from motion_capture_tracking)
and republishes individual drone poses as PoseStamped on /{drone_id}/pose

This is a workaround for when the Crazyflie's onboard logging isn't 
publishing pose data (e.g., due to radio bandwidth saturation with mocap).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped
from motion_capture_tracking_interfaces.msg import NamedPoseArray


class PoseBridgeNode(Node):
    """Bridge node to convert /poses to individual PoseStamped topics."""
    
    def __init__(self):
        super().__init__('pose_bridge_node')
        
        # Declare parameters
        self.declare_parameter('drone_ids', ['cf231'])
        self.declare_parameter('input_topic', '/poses')
        
        # Get parameters
        self.drone_ids = self.get_parameter('drone_ids').value
        input_topic = self.get_parameter('input_topic').value
        
        # Create publishers for each drone
        self.pose_publishers = {}
        for drone_id in self.drone_ids:
            topic = f'/{drone_id}/pose'
            self.pose_publishers[drone_id] = self.create_publisher(PoseStamped, topic, 10)
            self.get_logger().info(f'Publishing to: {topic}')
        
        # Subscribe to /poses with matching QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.subscription = self.create_subscription(
            NamedPoseArray,
            input_topic,
            self._poses_callback,
            qos_profile
        )
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Pose Bridge Node Ready')
        self.get_logger().info(f'  Input: {input_topic}')
        self.get_logger().info(f'  Drones: {self.drone_ids}')
        self.get_logger().info('=' * 50)
    
    def _poses_callback(self, msg: NamedPoseArray):
        """Convert NamedPoseArray to individual PoseStamped messages."""
        for named_pose in msg.poses:
            drone_id = named_pose.name
            
            if drone_id in self.pose_publishers:
                # Create PoseStamped message
                pose_msg = PoseStamped()
                pose_msg.header = msg.header
                pose_msg.pose = named_pose.pose
                
                # Publish
                self.pose_publishers[drone_id].publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
