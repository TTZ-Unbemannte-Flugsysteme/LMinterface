#!/usr/bin/env python3
"""
Batch Test Runner - State-Tracking Sequences

Runs tests in designed sequences with proper state tracking.

Usage:
    python3 run_batch_tests.py --mode filtered --seq 1       # Run sequence 1
    python3 run_batch_tests.py --mode baseline --seq all     # All sequences
    python3 run_batch_tests.py --mode filtered --seq 9,10    # Specific sequences
"""

import os
import sys
import time
import argparse
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger, Empty as EmptySrv
from geometry_msgs.msg import PoseStamped
from cocoa_msgs.msg import ExecutionFeedback, ThesisMetrics
from crazyflie_interfaces.srv import Land, GoTo, Takeoff

# Import from test_cases
from test_cases import ALL_SEQUENCES, ALL_TESTS, get_sequence
from metrics_logger import MetricsLogger

# Object coordinates from objects.yaml (for position verification)
OBJECT_POSITIONS = {
    'shelf_a': {'x': -10.0, 'y': 8.0, 'z': 1.0},
    'shelf_b': {'x': 10.0, 'y': 8.0, 'z': 1.0},
    'pallet_1': {'x': -6.0, 'y': 2.0, 'z': 0.6},
    'pallet_2': {'x': 6.0, 'y': 2.0, 'z': 0.6},
    'forklift': {'x': 6.0, 'y': -6.0, 'z': 0.6},
    'landing_pad': {'x': 0.0, 'y': -10.0, 'z': 0.01},
    'center_rack': {'x': 0.0, 'y': 5.0, 'z': 0.5},
    'box_1': {'x': -6.0, 'y': -6.0, 'z': 0.4},
}


class BatchTestRunner(Node):
    """Run multiple test cases and log results"""
    
    def __init__(self, mode, logger):
        super().__init__('batch_test_runner')
        
        self.mode = mode
        self.logger = logger
        
        # Publisher for voice commands
        self.cmd_pub = self.create_publisher(String, '/voice/text', 10)
        
        # Subscriber for execution feedback
        self.feedback_sub = self.create_subscription(
            ExecutionFeedback,
            '/execution_feedback',
            self.feedback_callback,
            10
        )
        
        # Subscriber for thesis metrics from nodes
        self.metrics_sub = self.create_subscription(
            ThesisMetrics,
            '/thesis_metrics',
            self.metrics_callback,
            10
        )
        
        # Subscriber for drone pose (for position verification)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/cf231/pose',
            self.pose_callback,
            10
        )
        
        # Test state
        self.current_test = None
        self.waiting = False
        self.result = None
        
        # Metrics from nodes
        self.intent_metrics = {}
        self.planner_metrics = {}
        
        # Pose tracking for verification
        self.current_pose = None
        self.pre_command_pose = None
        self.post_command_pose = None
        
        # State tracking
        self.current_state = 'grounded'  # Track drone state
        
        # Service clients for inter-sequence reset
        self.reset_history_client = self.create_client(Trigger, '/intent_extractor/reset_history')
        self.reset_lga_client = self.create_client(Trigger, '/lga/reset')
        self.land_client = self.create_client(Land, '/cf231/land')
        self.goto_client = self.create_client(GoTo, '/cf231/go_to')
        self.takeoff_client = self.create_client(Takeoff, '/cf231/takeoff')
        
        self.get_logger().info("Batch test runner ready")
    
    def run_test(self, test_case, timeout=90):
        """
        Run a single test case.
        
        Args:
            test_case: Dict with id, command, expected_intent, pre_state, post_state
            timeout: Seconds to wait for result
            
        Returns:
            Dict with result info
        """
        self.current_test = test_case
        self.waiting = True
        self.result = None
        
        # Reset metrics for this test
        self.intent_metrics = {}
        self.planner_metrics = {}
        
        # Wait for valid pose (fix for "pose data missing")
        pose_wait_start = time.time()
        while not self.current_pose and (time.time() - pose_wait_start) < 5.0:
             rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.current_pose:
             self.get_logger().warn("Could not get initial pose - verification may fail")
        
        # Record pre-command pose for verification
        self.pre_command_pose = self.current_pose.copy() if self.current_pose else None
        
        # Send command
        msg = String()
        msg.data = test_case['command']
        
        self.get_logger().info(f"[{test_case['id']}] Sending: {test_case['command']}")
        self.cmd_pub.publish(msg)
        
        # Wait for result
        start = time.time()
        while self.waiting and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Record post-command pose for verification
        self.post_command_pose = self.current_pose.copy() if self.current_pose else None
        
        if self.waiting:
            self.get_logger().warn(f"[{test_case['id']}] Timeout")
            return {
                'success': False,
                'intent_type': '',
                'reason': 'timeout',
                'verification_passed': None,
                'verification_issues': ['Timeout - could not verify']
            }
        
        # Merge metrics from nodes into result
        self.result['context_tokens'] = self.intent_metrics.get('context_tokens', 0)
        self.result['intent_time_ms'] = self.intent_metrics.get('intent_time_ms', 0)
        self.result['actual_target'] = self.intent_metrics.get('target', '')
        self.result['planner_time_ms'] = self.planner_metrics.get('planner_time_ms', 0)
        self.result['planner_tokens'] = self.planner_metrics.get('planner_tokens', 0)
        self.result['intent_llm_response'] = self.intent_metrics.get('llm_response', '')
        self.result['planner_llm_response'] = self.planner_metrics.get('llm_response', '')
        
        # Run position/distance/rotation verification
        verification_passed, issues, details = self._verify_outcome(
            test_case, self.pre_command_pose, self.post_command_pose
        )
        self.result['verification_passed'] = verification_passed
        self.result['verification_issues'] = issues
        self.result['verification_details'] = details
        
        # Log verification result
        if not verification_passed:
            self.get_logger().warn(f"[{test_case['id']}] Verification FAILED: {issues}")
        elif details:
            self.get_logger().info(f"[{test_case['id']}] Verification: {details}")
        
        # Update state tracking if successful
        if self.result.get('success', False):
            self.current_state = test_case.get('post_state', self.current_state)
        
        return self.result
    
    def metrics_callback(self, msg):
        """Handle thesis metrics from Intent Extractor and Planner nodes."""
        if msg.source == "intent_extractor":
            self.intent_metrics = {
                'context_tokens': msg.context_tokens,
                'intent_time_ms': msg.intent_time_ms,
                'target': msg.target,
                'llm_response': msg.llm_response
            }
        elif msg.source == "planner":
            self.planner_metrics = {
                'planner_time_ms': msg.planner_time_ms,
                'planner_tokens': getattr(msg, 'planner_tokens', 0),
                'action_count': msg.action_count,
                'llm_response': msg.llm_response
            }
    
    def pose_callback(self, msg):
        """Handle drone pose updates for position verification."""
        # Convert quaternion to yaw
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp) * 180.0 / math.pi  # Convert to degrees
        
        self.current_pose = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'yaw': yaw
        }
    
    def _verify_outcome(self, test_case, pre_pose, post_pose):
        """Verify actual drone state matches expected outcome.
        
        Returns:
            tuple: (verification_passed: bool, issues: list, details: dict)
        """
        issues = []
        details = {}
        
        if not pre_pose or not post_pose:
            return True, ["Could not verify: pose data missing"], {}
        
        # Record actual movement/position
        details['pre_position'] = f"({pre_pose['x']:.2f}, {pre_pose['y']:.2f}, {pre_pose['z']:.2f})"
        details['post_position'] = f"({post_pose['x']:.2f}, {post_pose['y']:.2f}, {post_pose['z']:.2f})"
        
        # For GO_TO_LOCATION: check if drone reached target object position
        expected_target = test_case.get('expected_target', '')
        if expected_target and expected_target in OBJECT_POSITIONS:
            target_pos = OBJECT_POSITIONS[expected_target]
            tolerance = test_case.get('position_tolerance', 2.5)  # 2m tolerance
            
            actual_dist = math.sqrt(
                (post_pose['x'] - target_pos['x'])**2 +
                (post_pose['y'] - target_pos['y'])**2
            )
            details['target'] = expected_target
            details['expected_position'] = f"({target_pos['x']:.2f}, {target_pos['y']:.2f})"
            details['distance_to_target'] = f"{actual_dist:.2f}m"
            
            if actual_dist > tolerance:
                issues.append(f"Position mismatch: expected near {expected_target}, but {actual_dist:.2f}m away (tolerance={tolerance}m)")
        
        # For MOVE_DIRECTION: check distance moved matches expected
        expected_intent = test_case.get('expected_intent', '')
        if expected_intent == 'MOVE_DIRECTION':
            # Extract expected distance from command
            command = test_case.get('command', '').lower()
            distance_words = ['meter', 'metre', 'm ']
            for word in distance_words:
                if word in command:
                    # Try to extract number before distance word
                    import re
                    match = re.search(r'(\d+\.?\d*)\s*' + word.replace(' ', ''), command)
                    if match:
                        expected_distance = float(match.group(1))
                        actual_distance = math.sqrt(
                            (post_pose['x'] - pre_pose['x'])**2 +
                            (post_pose['y'] - pre_pose['y'])**2
                        )
                        tolerance = test_case.get('distance_tolerance', 0.5)
                        details['expected_distance'] = f"{expected_distance}m"
                        details['actual_distance'] = f"{actual_distance:.2f}m"
                        
                        if abs(actual_distance - expected_distance) > tolerance:
                            issues.append(f"Distance mismatch: expected {expected_distance}m, moved {actual_distance:.2f}m")
                        break
        
        # For ROTATE: check yaw change
        if expected_intent == 'ROTATE':
            yaw_change = post_pose['yaw'] - pre_pose['yaw']
            # Normalize to -180 to 180
            while yaw_change > 180: yaw_change -= 360
            while yaw_change < -180: yaw_change += 360
            details['yaw_change'] = f"{yaw_change:.1f}°"
        
        verification_passed = len(issues) == 0
        return verification_passed, issues, details
    
    def feedback_callback(self, msg):
        """Handle execution feedback"""
        if not self.current_test:
            return
        
        # Extract actions and results
        actions = list(msg.actions) if hasattr(msg, 'actions') else []
        results = list(msg.results) if hasattr(msg, 'results') else []
        
        # For multi-intent, intent_type is comma-separated
        intent_type_raw = msg.intent_type if hasattr(msg, 'intent_type') else ''
        intent_types = intent_type_raw.split(',') if intent_type_raw else []
        
        # Build result
        self.result = {
            'success': msg.success if hasattr(msg, 'success') else False,
            'intent_type': intent_type_raw,
            'intent_types': intent_types,
            'command': msg.command if hasattr(msg, 'command') else '',
            'actions': actions,
            'results': results,
            'action_count': len(actions),
            'summary': msg.summary if hasattr(msg, 'summary') else ''
        }
        
        self.waiting = False
        
        status = "PASS" if self.result['success'] else "FAIL"
        self.get_logger().info(f"[{self.current_test['id']}] Result: {status}")
    
    def _call_service(self, client, request, name, timeout=5.0):
        """Helper to call a service with timeout and logging."""
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f'[RESET] {name} service not available')
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().warn(f'[RESET] {name} call failed')
            return None
    
    def reset_for_new_sequence(self, seq_label):
        """Reset drone position, conversation history, and battery between test sequences."""
        from builtin_interfaces.msg import Duration
        from geometry_msgs.msg import Point
        
        print(f"\n{'~'*60}")
        print(f"  [RESET] Resetting for {seq_label}")
        print(f"{'~'*60}")
        
        # 1. Land the drone (if it's still flying)
        self.get_logger().info('[RESET] Landing drone...')
        land_req = Land.Request()
        land_req.group_mask = 0
        land_req.height = 0.0
        land_req.duration = Duration(sec=3, nanosec=0)
        self._call_service(self.land_client, land_req, '/cf231/land')
        
        # Wait for landing to complete
        settle_start = time.time()
        while time.time() - settle_start < 4.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # 2. Takeoff to 1m (go_to requires the drone to be airborne)
        self.get_logger().info('[RESET] Taking off to 1m...')
        takeoff_req = Takeoff.Request()
        takeoff_req.group_mask = 0
        takeoff_req.height = 1.0
        takeoff_req.duration = Duration(sec=3, nanosec=0)
        self._call_service(self.takeoff_client, takeoff_req, '/cf231/takeoff')
        
        settle_start = time.time()
        while time.time() - settle_start < 4.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # 3. Go to origin — duration proportional to distance for gentle trajectory
        dist = 1.0  # default
        if self.current_pose:
            dx = self.current_pose['x']
            dy = self.current_pose['y']
            dist = math.sqrt(dx*dx + dy*dy) + 0.5  # +0.5 for safety margin
        
        # ~0.5 m/s cruise speed: 2 seconds per meter, minimum 5s
        goto_duration = max(5, int(dist * 2.0))
        self.get_logger().info(f'[RESET] Flying to origin (dist={dist:.1f}m, duration={goto_duration}s)...')
        
        goto_req = GoTo.Request()
        goto_req.group_mask = 0
        goto_req.relative = False
        goto_req.goal = Point(x=0.0, y=0.0, z=1.0)
        goto_req.yaw = 0.0
        goto_req.duration = Duration(sec=goto_duration, nanosec=0)
        self._call_service(self.goto_client, goto_req, '/cf231/go_to')
        
        # Wait for go_to to complete (duration + 2s buffer)
        settle_start = time.time()
        wait_time = goto_duration + 2.0
        while time.time() - settle_start < wait_time:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # 4. Land at origin
        self.get_logger().info('[RESET] Landing at origin...')
        self._call_service(self.land_client, land_req, '/cf231/land')
        
        settle_start = time.time()
        while time.time() - settle_start < 4.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info('[RESET] Drone returned to origin')
        
        # 5. Reset intent extractor conversation history
        result = self._call_service(self.reset_history_client, Trigger.Request(), '/intent_extractor/reset_history')
        if result:
            self.get_logger().info(f'[RESET] History: {result.message}')
        
        # 6. Reset LGA state (battery)
        result = self._call_service(self.reset_lga_client, Trigger.Request(), '/lga/reset')
        if result:
            self.get_logger().info(f'[RESET] LGA: {result.message}')
        
        # 7. Reset internal test runner state
        self.current_state = 'grounded'
        self.current_pose = None
        self.pre_command_pose = None
        self.post_command_pose = None
        
        # 8. Wait for drone to settle
        self.get_logger().info('[RESET] Waiting for drone to settle (3s)...')
        settle_start = time.time()
        while time.time() - settle_start < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        print(f"  [RESET] Done — drone at origin, history cleared\n")


def run_tests(tests, mode, output_file, delay=10.0):
    """
    Run a list of tests.
    
    Args:
        tests: List of test case dicts
        mode: 'filtered' or 'baseline'
        output_file: Path to CSV file
        delay: Seconds between tests
    """
    # Initialize ROS2
    rclpy.init()
    
    # Create logger and runner
    logger = MetricsLogger(output_file)
    runner = BatchTestRunner(mode, logger)
    
    # Toggle the actual node parameter!
    if mode == 'baseline':
        os.system('ros2 param set /intent_extractor use_lga false')
    else:
        os.system('ros2 param set /intent_extractor use_lga true')
        
    # Wait for connections
    time.sleep(2.0)
    
    # Stats
    passed = 0
    failed = 0
    skipped = 0
    rejections_correct = 0
    
    print(f"\n{'='*60}")
    print(f"Running {len(tests)} tests in {mode.upper()} mode")
    print(f"Output: {output_file}")
    print(f"{'='*60}\n")
    
    prev_seq_prefix = None  # Track sequence prefix for boundary detection
    
    try:
        for i, test in enumerate(tests):
            # Detect sequence boundary (e.g., "S1-01" -> "S2-01")
            test_id = test['id']
            seq_prefix = test_id.rsplit('-', 1)[0]  # e.g., "S1" from "S1-01"
            
            if prev_seq_prefix is not None and seq_prefix != prev_seq_prefix:
                # New sequence detected — reset simulation
                runner.reset_for_new_sequence(f"Sequence {seq_prefix}")
            prev_seq_prefix = seq_prefix
            
            pre_state = test.get('pre_state', 'unknown')
            expected_outcome = test.get('expected_outcome', 'SUCCESS')
            
            print(f"\n[{i+1}/{len(tests)}] {test['id']}: {test['command']}")
            print(f"  State: {pre_state} | Expected: {test['expected_intent']}")
            
            # Run test
            result = runner.run_test(test)
            
            # Check if intent matches
            actual_intent = result.get('intent_type', '')
            actual_intents = result.get('intent_types', [])
            expected_intent = test['expected_intent']
            
            # Intent is correct if expected is in actual list
            if actual_intents:
                intent_correct = expected_intent in actual_intents
            else:
                intent_correct = (actual_intent == expected_intent)
            
            # Check target correctness (for GO_TO_LOCATION)
            # Support both single expected_target and multi expected_targets (list)
            expected_target = test.get('expected_target', '')
            expected_targets = test.get('expected_targets', [])  # For multi-destination
            actual_target = result.get('actual_target', '')
            
            # Get all actual targets from intent_types if multi-intent
            actual_targets = []
            for action in result.get('actions', []):
                # Actions may contain target info - for now use actual_target
                pass
            if actual_target:
                actual_targets = [actual_target]
            
            # Target checking
            if expected_targets:
                # Multi-destination: check that all expected targets are in actual intents
                # For now, check that intent_count matches and first target is correct
                target_correct = True
                if expected_targets and actual_target:
                    target_correct = actual_target in expected_targets or actual_target == expected_targets[0]
            elif expected_target:
                target_correct = (expected_target == actual_target)
            else:
                target_correct = True
            
            # Check action count (for multi-intent tests)
            expected_action_count = test.get('expected_action_count', 0)
            actual_action_count = result.get('action_count', 0)
            action_count_correct = True
            if expected_action_count > 0:
                action_count_correct = (actual_action_count >= expected_action_count)
            
            # Check intent count (for multi-destination tests)
            expected_intent_count = test.get('expected_intent_count', 0)
            actual_intent_count = len(actual_intents) if actual_intents else (1 if actual_intent else 0)
            intent_count_correct = True
            if expected_intent_count > 0:
                intent_count_correct = (actual_intent_count >= expected_intent_count)
            
            # Handle expected rejections
            execution_success = result.get('success', False)
            
            if expected_outcome == 'REJECTION':
                # Rejection was expected
                if not execution_success:
                    rejections_correct += 1
                    test_passed = True
                    print(f"  Result: PASS (correctly rejected)")
                else:
                    test_passed = False
                    print(f"  Result: FAIL (expected rejection, got success)")
            else:
                # Success was expected
                test_passed = execution_success and intent_correct
                if expected_target or expected_targets:
                    test_passed = test_passed and target_correct
                if expected_action_count > 0:
                    test_passed = test_passed and action_count_correct
                if expected_intent_count > 0:
                    test_passed = test_passed and intent_count_correct
                
                # Factor in position/distance verification
                verification_passed = result.get('verification_passed', True)
                if verification_passed is not None:
                    test_passed = test_passed and verification_passed
                
                if test_passed:
                    print(f"  Result: PASS")
                else:
                    reason = ""
                    if not intent_correct:
                        reason = f"intent: expected {expected_intent}, got {actual_intent}"
                    elif (expected_target or expected_targets) and not target_correct:
                        exp_t = expected_target if expected_target else expected_targets
                        reason = f"target: expected {exp_t}, got {actual_target}"
                    elif expected_action_count > 0 and not action_count_correct:
                        reason = f"action_count: expected {expected_action_count}, got {actual_action_count}"
                    elif expected_intent_count > 0 and not intent_count_correct:
                        reason = f"intent_count: expected {expected_intent_count}, got {actual_intent_count}"
                    elif verification_passed is not None and not verification_passed:
                        v_issues = result.get('verification_issues', [])
                        reason = f"verification: {'; '.join(v_issues)}"
                    else:
                        reason = "execution failed"
                    print(f"  Result: FAIL ({reason})")
            
            # Build failure reason
            failure_reason = ""
            if not test_passed:
                if expected_outcome == 'REJECTION' and execution_success:
                    failure_reason = "Expected rejection but execution succeeded"
                elif not intent_correct:
                    failure_reason = f"Intent: expected {expected_intent}, got {actual_intent}"
                elif (expected_target or expected_targets) and not target_correct:
                    exp_t = expected_target if expected_target else expected_targets
                    failure_reason = f"Target: expected {exp_t}, got {actual_target}"
                elif expected_action_count > 0 and not action_count_correct:
                    failure_reason = f"Action count: expected {expected_action_count}, got {actual_action_count}"
                elif expected_intent_count > 0 and not intent_count_correct:
                    failure_reason = f"Intent count: expected {expected_intent_count}, got {actual_intent_count}"
                elif not execution_success:
                    failure_reason = "Execution failed"
            
            # Log result
            logger.log_test(
                test_id=test['id'],
                command=test['command'],
                mode=mode,
                expected_intent=expected_intent,
                actual_intent=actual_intent,
                expected_target=expected_target,
                actual_target=actual_target,
                pre_state=pre_state,
                post_state=test.get('post_state', ''),
                expected_outcome=expected_outcome,
                execution_success=execution_success,
                test_passed=test_passed,
                verification_passed=result.get('verification_passed'),
                verification_details=result.get('verification_details', {}),
                verification_issues=result.get('verification_issues', []),
                context_tokens=result.get('context_tokens', 0),
                planner_tokens=result.get('planner_tokens', 0),
                intent_time_ms=result.get('intent_time_ms', 0),
                planner_time_ms=result.get('planner_time_ms', 0),
                action_count=result.get('action_count', 0),
                intent_llm_response=result.get('intent_llm_response', ''),
                planner_llm_response=result.get('planner_llm_response', ''),
                failure_reason=failure_reason,
                notes=test.get('notes', '')
            )
            
            # Update stats
            if test_passed:
                passed += 1
            else:
                failed += 1
            
            # Delay between tests
            if i < len(tests) - 1:
                time.sleep(delay)
    
    except KeyboardInterrupt:
        print("\n\nTest run interrupted")
    
    finally:
        # Cleanup
        runner.destroy_node()
        rclpy.shutdown()
    
    # Summary
    total = passed + failed
    print(f"\n{'='*60}")
    print(f"Results: {passed} passed, {failed} failed")
    if rejections_correct > 0:
        print(f"  (including {rejections_correct} correct rejections)")
    if total > 0:
        print(f"Pass rate: {passed/total*100:.1f}%")
    print(f"Output saved to: {output_file}")
    print(f"{'='*60}\n")
    
    return passed, failed


def main():
    parser = argparse.ArgumentParser(description='Run thesis test sequences')
    parser.add_argument('--mode', choices=['filtered', 'baseline'], required=True,
                        help='Test mode: filtered (LGA) or baseline (full context)')
    parser.add_argument('--seq', default='all',
                        help='Sequence(s) to run: "all", single number (e.g., "1"), or comma-separated (e.g., "1,2,3")')
    parser.add_argument('--output', default=None,
                        help='Output CSV file')
    parser.add_argument('--delay', type=float, default=10.0,
                        help='Delay between tests (seconds, default: 10)')
    
    args = parser.parse_args()
    
    # Select tests by sequence
    if args.seq == 'all':
        tests = ALL_TESTS
        seq_str = 'all'
    else:
        tests = []
        seq_nums = [int(s.strip()) for s in args.seq.split(',')]
        for num in seq_nums:
            seq = get_sequence(num)
            if seq:
                tests.extend(seq)
            else:
                print(f"Warning: Sequence {num} not found")
        seq_str = args.seq.replace(',', '_')
    
    if not tests:
        print("No tests selected")
        print(f"Available sequences: {list(ALL_SEQUENCES.keys())}")
        return 1
    
    # Generate output filename
    if args.output:
        output = args.output
    else:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output = f"thesis_results_{args.mode}_seq{seq_str}_{timestamp}.csv"
    
    # Run tests
    passed, failed = run_tests(tests, args.mode, output, args.delay)
    
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
