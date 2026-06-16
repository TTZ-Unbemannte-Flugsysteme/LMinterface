#!/usr/bin/env python3
"""
Metrics Logger for Thesis Experiments
Logs test results to CSV file with state-aware outcome tracking.

Columns:
- test_passed: Did the test pass overall (considering expected_outcome)?
- expected_outcome: 'SUCCESS' or 'REJECTION'
- execution_success: Did execution complete (from Action Server)?
"""

import os
import csv
from datetime import datetime


class MetricsLogger:
    """CSV logger for thesis experiment metrics"""
    
    def __init__(self, csv_path):
        """
        Create a new metrics logger.
        
        Args:
            csv_path: Path to CSV file (will be created if doesn't exist)
        """
        self.csv_path = csv_path
        
        # Column headers - updated for state-aware testing
        self.headers = [
            'timestamp',
            'test_id',
            'sequence',           # Which sequence this test belongs to
            'command',
            'mode',
            'pre_state',          # Drone state before command
            'post_state',         # Drone state after command
            'expected_intent',
            'actual_intent',
            'intent_correct',
            'expected_target',
            'actual_target',
            'target_correct',
            'expected_outcome',   # 'SUCCESS' or 'REJECTION'
            'execution_success',  # Did execution complete? (from Action Server)
            'test_passed',        # Overall: did test pass considering expected_outcome?
            'verification_passed', # Did position/distance/rotation verification pass?
            'pre_position',       # Drone position before command
            'post_position',      # Drone position after command
            'distance_to_target', # Distance from expected target (for GO_TO_LOCATION)
            'actual_distance',    # Actual distance moved (for MOVE_DIRECTION)
            'verification_issues', # Any issues found during verification
            'context_tokens',
            'planner_tokens',
            'intent_time_ms',
            'planner_time_ms',
            'action_count',
            'intent_llm_response',
            'planner_llm_response',
            'failure_reason',
            'notes'
        ]
        
        # Create CSV if it doesn't exist
        if not os.path.exists(csv_path):
            self._create_csv()
    
    def _create_csv(self):
        """Create CSV file with headers"""
        directory = os.path.dirname(self.csv_path)
        if directory and not os.path.exists(directory):
            os.makedirs(directory)
        
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(self.headers)
        
        print(f"Created metrics file: {self.csv_path}")
    
    def log_test(self, test_id, command, mode, 
                 expected_intent, actual_intent,
                 expected_target='', actual_target='',
                 pre_state='', post_state='',
                 expected_outcome='SUCCESS',
                 execution_success=False,
                 test_passed=None,  # Pass in pre-calculated result from runner
                 verification_passed=None,
                 verification_details=None,
                 verification_issues=None,
                 context_tokens=0, planner_tokens=0, intent_time_ms=0, planner_time_ms=0,
                 action_count=0, 
                 intent_llm_response='', planner_llm_response='',
                 failure_reason='', notes=''):
        """
        Log a test result.
        
        Args:
            test_id: Test ID like 'S1-01'
            command: The voice command
            mode: 'filtered' or 'baseline'
            expected_intent: What intent we expected
            actual_intent: What the LLM extracted
            expected_target: Expected target for GO_TO_LOCATION
            actual_target: Actual target extracted
            pre_state: Drone state before command ('grounded' or 'flying')
            post_state: Drone state after command
            expected_outcome: 'SUCCESS' or 'REJECTION'
            execution_success: Did execution complete (from Action Server)?
            context_tokens: Number of context tokens
            intent_time_ms: Intent extraction time
            planner_time_ms: Planner time
            action_count: Number of actions
            intent_llm_response: Raw intent LLM response
            planner_llm_response: Raw planner LLM response
            failure_reason: Why it failed
            notes: Any notes
        """
        # Extract sequence number from test_id (e.g., 'S1-01' -> '1')
        sequence = test_id.split('-')[0].replace('S', '') if test_id.startswith('S') else '0'
        
        # Check if intent matches
        intent_correct = (expected_intent == actual_intent)
        
        # Check if target matches (only for GO_TO_LOCATION)
        target_correct = True
        if expected_target:
            target_correct = (expected_target == actual_target)
        
        # Use passed-in test_passed if provided, otherwise calculate
        if test_passed is None:
            # Calculate test_passed based on expected_outcome (fallback)
            if expected_outcome == 'REJECTION':
                # For expected rejection: pass if execution did NOT succeed
                test_passed = not execution_success
            else:
                # For expected success: pass if execution succeeded AND intent correct
                test_passed = execution_success and intent_correct
                if expected_target:
                    test_passed = test_passed and target_correct
        
        # Extract verification details
        v_details = verification_details or {}
        pre_position = v_details.get('pre_position', '')
        post_position = v_details.get('post_position', '')
        distance_to_target = v_details.get('distance_to_target', '')
        actual_distance = v_details.get('actual_distance', '')
        v_issues = '; '.join(verification_issues) if verification_issues else ''
        
        # Format verification_passed
        if verification_passed is None:
            v_passed_str = ''
        else:
            v_passed_str = 'true' if verification_passed else 'false'
        
        # Build row
        row = [
            datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            test_id,
            sequence,
            command,
            mode,
            pre_state,
            post_state,
            expected_intent,
            actual_intent,
            'true' if intent_correct else 'false',
            expected_target,
            actual_target,
            'true' if target_correct else 'false',
            expected_outcome,
            'true' if execution_success else 'false',
            'true' if test_passed else 'false',
            v_passed_str,
            pre_position,
            post_position,
            distance_to_target,
            actual_distance,
            v_issues,
            context_tokens,
            planner_tokens,
            intent_time_ms,
            planner_time_ms,
            action_count,
            intent_llm_response,
            planner_llm_response,
            failure_reason,
            notes
        ]
        
        # Append to CSV
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)
        
        # Print summary
        status = 'PASS' if test_passed else 'FAIL'
        outcome_note = ' (correct rejection)' if (expected_outcome == 'REJECTION' and test_passed) else ''
        print(f"[{test_id}] {status}{outcome_note} - {command}")


# Simple test
if __name__ == '__main__':
    logger = MetricsLogger('/tmp/test_metrics.csv')
    
    # Test normal success
    logger.log_test(
        test_id='S1-01',
        command='Take off',
        mode='filtered',
        expected_intent='TAKEOFF',
        actual_intent='TAKEOFF',
        pre_state='grounded',
        post_state='flying',
        expected_outcome='SUCCESS',
        execution_success=True,
        context_tokens=1228,
        intent_time_ms=150,
        action_count=1
    )
    
    # Test expected rejection that was correctly rejected
    logger.log_test(
        test_id='S1-07',
        command='Land now',
        mode='filtered',
        expected_intent='LAND',
        actual_intent='LAND',
        pre_state='grounded',
        post_state='grounded',
        expected_outcome='REJECTION',
        execution_success=False,  # Correctly rejected
        context_tokens=1390,
        intent_time_ms=970,
        action_count=0,
        notes='Correct: rejected landing when grounded'
    )
    
    # Test expected rejection that was NOT rejected (FAIL case)
    logger.log_test(
        test_id='S1-07-BAD',
        command='Land now',
        mode='filtered',
        expected_intent='LAND',
        actual_intent='LAND',
        pre_state='grounded',
        post_state='grounded',
        expected_outcome='REJECTION',
        execution_success=True,  # Should have rejected but didn't!
        context_tokens=1390,
        intent_time_ms=970,
        action_count=2,
        failure_reason='LLM did takeoff+land instead of rejecting',
        notes='Wrong: should have rejected landing when grounded'
    )
    
    print(f"\nTest CSV created at /tmp/test_metrics.csv")
