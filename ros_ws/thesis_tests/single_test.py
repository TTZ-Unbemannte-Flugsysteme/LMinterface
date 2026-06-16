#!/usr/bin/env python3
"""
Simple Single Test
Sends one voice command and shows the result.
Use this to verify the system is working before running full tests.

Usage:
    python3 single_test.py "take off"
    python3 single_test.py "go to shelf A"
"""

import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cocoa_msgs.msg import ExecutionFeedback


class SingleTestNode(Node):
    """Simple node to send one command and wait for result"""
    
    def __init__(self):
        super().__init__('single_test')
        
        # Publisher for voice commands
        self.cmd_pub = self.create_publisher(String, '/voice/text', 10)
        
        # Subscriber for execution feedback
        self.feedback_sub = self.create_subscription(
            ExecutionFeedback,
            '/execution_feedback',
            self.feedback_callback,
            10
        )
        
        self.result_received = False
        self.get_logger().info("Single test node ready")
    
    def send_command(self, command):
        """Send a voice command"""
        msg = String()
        msg.data = command
        
        # Wait for publisher to connect to subscribers
        self.get_logger().info("Waiting for subscribers...")
        wait_count = 0
        while self.cmd_pub.get_subscription_count() == 0 and wait_count < 50:
            time.sleep(0.1)
            wait_count += 1
        
        if self.cmd_pub.get_subscription_count() == 0:
            self.get_logger().warn("No subscribers found for /voice/text!")
            self.get_logger().warn("Make sure intent_extractor_node is running")
        
        self.get_logger().info(f"Sending command: '{command}'")
        self.cmd_pub.publish(msg)
    
    def feedback_callback(self, msg):
        """Handle execution feedback"""
        print("\n" + "="*50)
        print("EXECUTION FEEDBACK RECEIVED")
        print("="*50)
        
        if hasattr(msg, 'success'):
            status = "SUCCESS" if msg.success else "FAILED"
            print(f"Status: {status}")
        
        if hasattr(msg, 'intent_type') and msg.intent_type:
            print(f"Intent: {msg.intent_type}")
        
        if hasattr(msg, 'command') and msg.command:
            print(f"Command: {msg.command}")
        
        if hasattr(msg, 'details') and msg.details:
            print(f"Actions: {msg.details}")
        
        print("="*50 + "\n")
        
        self.result_received = True
    
    def wait_for_result(self, timeout=45):
        """Wait for result with timeout"""
        start = time.time()
        while not self.result_received and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.result_received:
            print("\nTimeout - no feedback received")
            print("Make sure the planner node is running")
        
        return self.result_received


def main():
    # Get command from argument or prompt
    if len(sys.argv) > 1:
        command = " ".join(sys.argv[1:])
    else:
        print("\nEnter a voice command to test:")
        print("Examples: 'take off', 'go to shelf A', 'move forward 1 meter'")
        command = input("> ").strip()
    
    if not command:
        print("No command provided")
        return
    
    # Initialize ROS2
    rclpy.init()
    
    # Create node and run test
    node = SingleTestNode()
    
    try:
        node.send_command(command)
        node.wait_for_result(timeout=45)
    except KeyboardInterrupt:
        print("\nTest cancelled")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
