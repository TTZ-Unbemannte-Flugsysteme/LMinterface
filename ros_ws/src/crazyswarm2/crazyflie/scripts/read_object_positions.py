#!/usr/bin/env python3
"""
OptiTrack Object Position Reader

Reads rigid body positions from /poses topic and prints them in a format
that can be directly copied into the EKG configuration.

Usage:
    ros2 run crazyflie read_object_positions.py
    
Or run directly:
    python3 read_object_positions.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from motion_capture_tracking_interfaces.msg import NamedPoseArray
import yaml


class ObjectPositionReader(Node):
    """Read and display rigid body positions from OptiTrack."""
    
    def __init__(self):
        super().__init__('object_position_reader')
        
        # Parameters
        self.declare_parameter('exclude_drones', ['cf231'])  # Don't show drone, only static objects
        self.declare_parameter('samples', 10)  # Average over N samples for accuracy
        
        self.exclude_drones = self.get_parameter('exclude_drones').value
        self.samples_needed = self.get_parameter('samples').value
        
        # Storage for averaging
        self.position_samples = {}  # {name: [[x,y,z], ...]}
        
        # Subscribe to /poses
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.subscription = self.create_subscription(
            NamedPoseArray,
            '/poses',
            self._poses_callback,
            qos_profile
        )
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('OptiTrack Object Position Reader')
        self.get_logger().info(f'  Excluding: {self.exclude_drones}')
        self.get_logger().info(f'  Averaging over {self.samples_needed} samples')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Waiting for rigid bodies on /poses...')
    
    def _poses_callback(self, msg: NamedPoseArray):
        """Collect position samples from OptiTrack."""
        for named_pose in msg.poses:
            name = named_pose.name
            
            # Skip drones
            if name in self.exclude_drones:
                continue
            
            x = named_pose.pose.position.x
            y = named_pose.pose.position.y
            z = named_pose.pose.position.z
            
            # Initialize if new object
            if name not in self.position_samples:
                self.position_samples[name] = []
                self.get_logger().info(f'Found new object: {name}')
            
            # Collect samples
            if len(self.position_samples[name]) < self.samples_needed:
                self.position_samples[name].append([x, y, z])
                
                if len(self.position_samples[name]) == self.samples_needed:
                    self._print_averaged_position(name)
    
    def _print_averaged_position(self, name):
        """Calculate and print the averaged position."""
        samples = self.position_samples[name]
        
        avg_x = sum(s[0] for s in samples) / len(samples)
        avg_y = sum(s[1] for s in samples) / len(samples)
        avg_z = sum(s[2] for s in samples) / len(samples)
        
        # Print in multiple formats
        self.get_logger().info('')
        self.get_logger().info(f'=== {name} ===')
        self.get_logger().info(f'  Position: x={avg_x:.4f}, y={avg_y:.4f}, z={avg_z:.4f}')
        
        # Print YAML format for EKG
        print(f'\n# YAML format for EKG config:')
        print(f'{name}:')
        print(f'  position:')
        print(f'    x: {avg_x:.4f}')
        print(f'    y: {avg_y:.4f}')
        print(f'    z: {avg_z:.4f}')
        print(f'  type: "obstacle"  # or "waypoint", "landing_zone", etc.')
        print()
        
        # Check if all objects are done
        all_done = all(
            len(samples) >= self.samples_needed 
            for samples in self.position_samples.values()
        )
        if all_done and len(self.position_samples) > 0:
            self._print_summary()
    
    def _print_summary(self):
        """Print complete summary of all objects."""
        print('\n' + '=' * 60)
        print('COMPLETE EKG CONFIGURATION')
        print('=' * 60)
        print('# Copy this into your EKG config file (e.g., ekg_config.yaml)')
        print('objects:')
        
        for name, samples in self.position_samples.items():
            avg_x = sum(s[0] for s in samples) / len(samples)
            avg_y = sum(s[1] for s in samples) / len(samples)
            avg_z = sum(s[2] for s in samples) / len(samples)
            
            print(f'  {name}:')
            print(f'    position: [{avg_x:.4f}, {avg_y:.4f}, {avg_z:.4f}]')
            print(f'    type: "obstacle"')
        
        print('=' * 60)
        print('Press Ctrl+C to exit')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPositionReader()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
