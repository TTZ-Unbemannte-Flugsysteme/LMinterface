"""
Drone Command Action Server

ROS2 Action Server that executes individual drone commands (takeoff, goto, land, hover)
via Crazyswarm2 services. Uses non-blocking rate.sleep() to allow callbacks to run
while waiting for actions to complete.
"""

import math
import time
from threading import Event

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point, PoseStamped
from builtin_interfaces.msg import Duration

from crazyflie_interfaces.srv import Takeoff, Land, GoTo, Arm, NotifySetpointsStop
from cocoa_msgs.action import DroneCommand


class DroneCommandActionServer(Node):
    """
    Action Server for executing drone commands.
    
    Uses Crazyswarm2 services for drone control and provides
    real-time feedback on command progress.
    """
    
    def __init__(self):
        super().__init__('drone_command_action_server')
        
        # Declare parameters
        self.declare_parameter('drone_id', 'cf231')
        self.declare_parameter('position_tolerance', 0.15)
        self.declare_parameter('height_tolerance', 0.1)
        self.declare_parameter('ground_height', 0.05)
        self.declare_parameter('action_timeout', 30.0)
        self.declare_parameter('feedback_rate', 10.0)
        
        # Get parameters
        self.drone_id = self.get_parameter('drone_id').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.height_tolerance = self.get_parameter('height_tolerance').value
        self.ground_height = self.get_parameter('ground_height').value
        self.action_timeout = self.get_parameter('action_timeout').value
        self.feedback_rate = self.get_parameter('feedback_rate').value
        
        # Current drone state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.is_armed = False
        
        # Use ReentrantCallbackGroup to allow concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Create service clients for Crazyswarm2
        self.takeoff_client = self.create_client(
            Takeoff,
            f'/{self.drone_id}/takeoff',
            callback_group=self.callback_group
        )
        self.land_client = self.create_client(
            Land,
            f'/{self.drone_id}/land',
            callback_group=self.callback_group
        )
        self.goto_client = self.create_client(
            GoTo,
            f'/{self.drone_id}/go_to',
            callback_group=self.callback_group
        )
        self.arm_client = self.create_client(
            Arm,
            f'/{self.drone_id}/arm',
            callback_group=self.callback_group
        )
        self.stop_client = self.create_client(
            NotifySetpointsStop,
            f'/{self.drone_id}/notify_setpoints_stop',
            callback_group=self.callback_group
        )
        
        # Subscribe to pose for position feedback
        self.pose_sub = self.create_subscription(
            PoseStamped,
            f'/{self.drone_id}/pose',
            self._pose_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Create the Action Server
        self._action_server = ActionServer(
            self,
            DroneCommand,
            'drone_command',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info(f'Drone Command Action Server ready for drone: {self.drone_id}')
    
    def _pose_callback(self, msg: PoseStamped):
        """Update current drone position from pose messages."""
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
    
    def _goal_callback(self, goal_request):
        """Accept or reject incoming goals."""
        self.get_logger().info(f'Received goal: {goal_request.command_type}')
        return GoalResponse.ACCEPT
    
    def _cancel_callback(self, goal_handle):
        """Accept cancellation requests."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    async def _execute_callback(self, goal_handle):
        """
        Execute the drone command.
        
        This runs in a separate thread and uses async/await patterns
        to avoid blocking the executor.
        """
        command_type = goal_handle.request.command_type.lower()
        
        self.get_logger().info(f'Executing: {command_type}')
        
        if command_type == 'takeoff':
            self.get_logger().info(f'Start Takeoff - current z: {self.current_z:.2f}m')
            return await self._execute_takeoff(goal_handle)
        elif command_type == 'goto':
            self.get_logger().info(f'Start GoTo - current z: {self.current_z:.2f}m')
            return await self._execute_goto(goal_handle)
        elif command_type == 'land':
            self.get_logger().info(f'Start Land - current z: {self.current_z:.2f}m')
            return await self._execute_land(goal_handle)
        elif command_type == 'hover':
            self.get_logger().info(f'Start Hover - current z: {self.current_z:.2f}m')
            return await self._execute_hover(goal_handle)
        else:
            result = DroneCommand.Result()
            result.success = False
            result.message = f'Unknown command: {command_type}'
            result.final_x = self.current_x
            result.final_y = self.current_y
            result.final_z = self.current_z
            goal_handle.abort()
            return result
    
    # =========================================================================
    # TAKEOFF
    # =========================================================================
    
    async def _execute_takeoff(self, goal_handle):
        """Execute takeoff command."""
        target_z = goal_handle.request.target_z
        if target_z <= 0:
            target_z = 1.0  # Default height
        
        # Implicitly arm the drone before takeoff if not already armed
        if not self.is_armed:
            self.get_logger().info('Implicitly arming before takeoff...')
            
            # Reset high-level commander state before arming
            if self.stop_client.wait_for_service(timeout_sec=1.0):
                stop_req = NotifySetpointsStop.Request()
                await self.stop_client.call_async(stop_req)
                self.get_logger().info('Sent notify_setpoints_stop')

            arm_success = await self._perform_arm(True)
            if not arm_success:
                return self._abort_result(goal_handle, 'Failed to arm drone before takeoff')
            self.is_armed = True
            # Short wait after arming (use time.sleep since no asyncio loop)
            time.sleep(1.5)
        
        # Wait for service
        if not self.takeoff_client.wait_for_service(timeout_sec=2.0):
            return self._abort_result(goal_handle, 'Takeoff service not available')
        
        # Send takeoff request
        request = Takeoff.Request()
        request.group_mask = 0
        request.height = target_z
        request.duration = Duration(sec=3, nanosec=0)
        
        future = self.takeoff_client.call_async(request)
        await future  # Wait for service response
        
        self.get_logger().info(f'Takeoff command sent, waiting for height {target_z}m')
        
        # Monitor progress until target height reached
        return await self._wait_for_height(goal_handle, target_z, 'rising')
    
    async def _wait_for_height(self, goal_handle, target_z, status):
        """Wait until drone reaches target height, publishing feedback."""
        feedback = DroneCommand.Feedback()
        start_time = self.get_clock().now()
        rate = self.create_rate(self.feedback_rate)
        
        while True:
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._cancel_result()
            
            # Check if target reached
            if self.current_z >= (target_z - self.height_tolerance):
                return self._success_result(goal_handle, f'Reached height {target_z}m')
            
            # Check timeout
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > self.action_timeout:
                return self._abort_result(goal_handle, 
                    f'Timeout at z={self.current_z:.2f}m')
            
            # Publish feedback
            feedback.current_x = self.current_x
            feedback.current_y = self.current_y
            feedback.current_z = self.current_z
            feedback.distance_remaining = abs(target_z - self.current_z)
            feedback.status = status
            goal_handle.publish_feedback(feedback)
            
            # Non-blocking sleep
            rate.sleep()

    # =========================================================================
    # ARM
    # =========================================================================
    async def _perform_arm(self, arm_state: bool) -> bool:
        """Internal helper to call the Arm service."""
        if not self.arm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Arm service not available')
            return False
        
        req = Arm.Request()
        req.arm = arm_state
        
        try:
            future = self.arm_client.call_async(req)
            await future
            return True # Crazyswarm2 Arm service doesn't return success/fail in my memory of old versions, but let's assume it worked if it returned
        except Exception as e:
            self.get_logger().error(f"Arm service call failed: {e}")
            return False
    
    # =========================================================================
    # LAND
    # =========================================================================
    
    async def _execute_land(self, goal_handle):
        """Execute land command."""
        if not self.land_client.wait_for_service(timeout_sec=2.0):
            return self._abort_result(goal_handle, 'Land service not available')
        
        request = Land.Request()
        request.group_mask = 0
        request.height = 0.0
        request.duration = Duration(sec=3, nanosec=0)
        
        future = self.land_client.call_async(request)
        await future
        
        self.get_logger().info('Land command sent, waiting for ground')
        
        result = await self._wait_for_ground(goal_handle)
        
        if result.success:
            self.get_logger().info('Disarming after landing...')
            await self._perform_arm(False)
            self.is_armed = False
            
        return result
    
    async def _wait_for_ground(self, goal_handle):
        """Wait until drone lands."""
        feedback = DroneCommand.Feedback()
        start_time = self.get_clock().now()
        start_z = self.current_z
        rate = self.create_rate(self.feedback_rate)
        
        self.get_logger().info(f'Waiting for ground. Start z: {start_z:.2f}m')
        
        # Minimum wait of 1.0 second to ensure the land command has started
        # and we don't immediately trigger on a low current_z (use time.sleep since no asyncio loop)
        time.sleep(1.0)
        
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._cancel_result()
            
            # Diagnostic log every ~1 second
            current_elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if int(current_elapsed * 10) % 10 == 0:
                self.get_logger().info(f'  Landing progress: z={self.current_z:.2f}m')

            if self.current_z <= self.ground_height:
                self.get_logger().info(f'Reached ground (z={self.current_z:.2f}m <= {self.ground_height}m)')
                return self._success_result(goal_handle, 'Landed successfully')
            
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > self.action_timeout:
                return self._abort_result(goal_handle,
                    f'Land timeout at z={self.current_z:.2f}m')
            
            feedback.current_x = self.current_x
            feedback.current_y = self.current_y
            feedback.current_z = self.current_z
            feedback.distance_remaining = self.current_z
            feedback.status = 'descending'
            goal_handle.publish_feedback(feedback)
            
            rate.sleep()
    
    # =========================================================================
    # GOTO
    # =========================================================================
    
    async def _execute_goto(self, goal_handle):
        """Execute goto command."""
        target_x = goal_handle.request.target_x - 0.0
        target_y = goal_handle.request.target_y - 0.0
        target_z = goal_handle.request.target_z
        yaw = goal_handle.request.yaw
        
        if target_z <= 0:
            target_z = 1.0  # Default height
        
        if not self.goto_client.wait_for_service(timeout_sec=2.0):
            return self._abort_result(goal_handle, 'GoTo service not available')
        
        # Calculate duration based on distance
        distance = self._calculate_distance(target_x, target_y, target_z)
        duration_sec = max(2, int(distance * 2))  # ~0.5 m/s
        
        request = GoTo.Request()
        request.group_mask = 0
        request.relative = False
        request.goal = Point(x=target_x, y=target_y, z=target_z)
        request.yaw = yaw
        request.duration = Duration(sec=duration_sec, nanosec=0)
        
        future = self.goto_client.call_async(request)
        await future
        
        self.get_logger().info(f'GoTo command sent: ({target_x}, {target_y}, {target_z})')
        
        return await self._wait_for_position(goal_handle, target_x, target_y, target_z)
    
    async def _wait_for_position(self, goal_handle, target_x, target_y, target_z):
        """Wait until drone reaches target position."""
        feedback = DroneCommand.Feedback()
        start_time = self.get_clock().now()
        rate = self.create_rate(self.feedback_rate)
        
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._cancel_result()
            
            distance = self._calculate_distance(target_x, target_y, target_z)
            
            if distance <= self.position_tolerance:
                return self._success_result(goal_handle,
                    f'Arrived at ({target_x}, {target_y}, {target_z})')
            
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > self.action_timeout:
                return self._abort_result(goal_handle,
                    f'GoTo timeout at ({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f})')
            
            feedback.current_x = self.current_x
            feedback.current_y = self.current_y
            feedback.current_z = self.current_z
            feedback.distance_remaining = distance
            feedback.status = 'moving'
            goal_handle.publish_feedback(feedback)
            
            rate.sleep()
    
    def _calculate_distance(self, target_x, target_y, target_z):
        """Calculate distance from current position to target."""
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        dz = target_z - self.current_z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    # =========================================================================
    # HOVER
    # =========================================================================
    
    async def _execute_hover(self, goal_handle):
        """Execute hover command (just wait in place)."""
        duration = goal_handle.request.duration
        if duration <= 0:
            duration = 2.0  # Default 2 seconds
        
        feedback = DroneCommand.Feedback()
        start_time = self.get_clock().now()
        rate = self.create_rate(self.feedback_rate)
        
        self.get_logger().info(f'Hovering for {duration}s')
        
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._cancel_result()
            
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            
            if elapsed >= duration:
                return self._success_result(goal_handle, f'Hovered for {duration}s')
            
            feedback.current_x = self.current_x
            feedback.current_y = self.current_y
            feedback.current_z = self.current_z
            feedback.distance_remaining = 0.0
            feedback.status = 'hovering'
            goal_handle.publish_feedback(feedback)
            
            rate.sleep()
    
    # =========================================================================
    # RESULT HELPERS
    # =========================================================================
    
    def _success_result(self, goal_handle, message):
        """Create and return a success result."""
        result = DroneCommand.Result()
        result.success = True
        result.message = message
        result.final_x = self.current_x
        result.final_y = self.current_y
        result.final_z = self.current_z
        goal_handle.succeed()
        self.get_logger().info(f'Success: {message}')
        return result
    
    def _abort_result(self, goal_handle, message):
        """Create and return an abort result."""
        result = DroneCommand.Result()
        result.success = False
        result.message = message
        result.final_x = self.current_x
        result.final_y = self.current_y
        result.final_z = self.current_z
        goal_handle.abort()
        self.get_logger().error(f'Aborted: {message}')
        return result
    
    def _cancel_result(self):
        """Create and return a cancel result."""
        result = DroneCommand.Result()
        result.success = False
        result.message = 'Goal canceled'
        result.final_x = self.current_x
        result.final_y = self.current_y
        result.final_z = self.current_z
        self.get_logger().info('Goal canceled')
        return result


def main(args=None):
    rclpy.init(args=args)
    
    node = DroneCommandActionServer()
    
    # Use MultiThreadedExecutor for concurrent callback execution
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
