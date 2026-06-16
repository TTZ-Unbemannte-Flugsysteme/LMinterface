#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from crazyflie_interfaces.srv import Takeoff, Land, Arm

class SimpleFlightTest(Node):
    def __init__(self):
        super().__init__('simple_flight_test')
        self.drone_id = 'cf231'
        
        # Clients
        self.takeoff_client = self.create_client(Takeoff, f'/{self.drone_id}/takeoff')
        self.land_client = self.create_client(Land, f'/{self.drone_id}/land')
        self.arm_client = self.create_client(Arm, f'/{self.drone_id}/arm')
        
    def wait_for_services(self):
        self.get_logger().info('Waiting for services...')
        if not self.takeoff_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Takeoff service not available!')
            return False
        if not self.land_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Land service not available!')
            return False
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Arm service not available!')
            return False
        self.get_logger().info('Services ready.')
        return True

    def run_test(self):
        if not self.wait_for_services():
            return

        # Arm
        self.get_logger().info('Arming...')
        arm_req = Arm.Request()
        arm_req.arm = True
        future = self.arm_client.call_async(arm_req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Armed.')
        time.sleep(1.0)

        # Takeoff
        self.get_logger().info('Taking off...')
        req = Takeoff.Request()
        req.group_mask = 0
        req.height = 1.0
        req.duration = Duration(seconds=3).to_msg()
        
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Takeoff command sent.')
        
        # Hover
        self.get_logger().info('Hovering for 5 seconds...')
        time.sleep(5.0)
        
        # Land
        self.get_logger().info('Landing...')
        req = Land.Request()
        req.group_mask = 0
        req.height = 0.0
        req.duration = Duration(seconds=3).to_msg()
        
        future = self.land_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Land command sent.')
        
        # Wait for land to finish approximately
        time.sleep(4.0)
        self.get_logger().info('Test complete.')

def main():
    rclpy.init()
    node = SimpleFlightTest()
    
    try:
        node.run_test()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
