"""
Metrics Logger for Thesis Experiments
Simple CSV logging for Intent Extractor and Planner metrics
"""

import os
import csv
from datetime import datetime


class MetricsLogger:
    """
    Simple CSV logger for thesis experiment metrics.
    Logs each test case result to a CSV file.
    """
    
    # CSV column headers
    HEADERS = [
        'timestamp',
        'test_id',
        'level',
        'command',
        'mode',
        'expected_intent',
        'actual_intent',
        'intent_correct',
        'expected_target',
        'actual_target',
        'target_correct',
        'context_tokens',
        'intent_inference_ms',
        'planner_inference_ms',
        'plan_valid',
        'action_count',
        'execution_success',
        'failure_reason',
        'total_time_ms',
        'notes'
    ]
    
    def __init__(self, csv_path):
        """
        Initialize the metrics logger.
        
        Args:
            csv_path: Path to the CSV file for logging
        """
        self.csv_path = csv_path
        self.current_test = {}  # Store current test data
        
        # Create CSV file with headers if it doesn't exist
        if not os.path.exists(csv_path):
            self._create_csv_file()
    
    def _create_csv_file(self):
        """Create the CSV file with headers"""
        # Make sure directory exists
        directory = os.path.dirname(self.csv_path)
        if directory and not os.path.exists(directory):
            os.makedirs(directory)
        
        # Write headers
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(self.HEADERS)
    
    def start_test(self, test_id, level, command, mode, expected_intent, expected_target=''):
        """
        Start recording a new test case.
        
        Args:
            test_id: Test case ID (e.g., 'L1-01')
            level: Test level (1-4)
            command: The voice command being tested
            mode: 'filtered' or 'baseline'
            expected_intent: What intent we expect
            expected_target: What target we expect (for GO_TO_LOCATION)
        """
        self.current_test = {
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'test_id': test_id,
            'level': level,
            'command': command,
            'mode': mode,
            'expected_intent': expected_intent,
            'expected_target': expected_target,
            'actual_intent': '',
            'actual_target': '',
            'intent_correct': False,
            'target_correct': False,
            'context_tokens': 0,
            'intent_inference_ms': 0,
            'planner_inference_ms': 0,
            'plan_valid': False,
            'action_count': 0,
            'execution_success': False,
            'failure_reason': '',
            'total_time_ms': 0,
            'notes': '',
            'start_time': datetime.now()
        }
    
    def log_intent_extraction(self, actual_intent, actual_target='', 
                               context_tokens=0, inference_ms=0):
        """
        Log intent extraction results.
        
        Args:
            actual_intent: What intent the LLM extracted
            actual_target: What target the LLM extracted
            context_tokens: Number of context tokens used
            inference_ms: LLM inference time in milliseconds
        """
        self.current_test['actual_intent'] = actual_intent
        self.current_test['actual_target'] = actual_target
        self.current_test['context_tokens'] = context_tokens
        self.current_test['intent_inference_ms'] = inference_ms
        
        # Check if intent is correct
        expected = self.current_test.get('expected_intent', '')
        self.current_test['intent_correct'] = (actual_intent == expected)
        
        # Check if target is correct (for GO_TO_LOCATION)
        expected_target = self.current_test.get('expected_target', '')
        if expected_target:
            self.current_test['target_correct'] = (actual_target == expected_target)
        else:
            self.current_test['target_correct'] = True  # No target expected
    
    def log_planning(self, inference_ms=0, plan_valid=False, action_count=0):
        """
        Log planner results.
        
        Args:
            inference_ms: Planner LLM inference time in milliseconds
            plan_valid: Whether the plan JSON was valid
            action_count: Number of actions in the plan
        """
        self.current_test['planner_inference_ms'] = inference_ms
        self.current_test['plan_valid'] = plan_valid
        self.current_test['action_count'] = action_count
    
    def log_execution(self, success=False, failure_reason=''):
        """
        Log execution results.
        
        Args:
            success: Whether execution was successful
            failure_reason: Reason for failure if any
        """
        self.current_test['execution_success'] = success
        self.current_test['failure_reason'] = failure_reason
    
    def add_note(self, note):
        """Add a note to the current test"""
        self.current_test['notes'] = note
    
    def finish_test(self):
        """
        Finish the current test and write to CSV.
        Returns the test data as a dictionary.
        """
        # Calculate total time
        if 'start_time' in self.current_test:
            elapsed = datetime.now() - self.current_test['start_time']
            self.current_test['total_time_ms'] = int(elapsed.total_seconds() * 1000)
        
        # Write to CSV
        self._write_row()
        
        # Return the test data
        result = dict(self.current_test)
        self.current_test = {}
        return result
    
    def _write_row(self):
        """Write current test data to CSV"""
        row = []
        for header in self.HEADERS:
            value = self.current_test.get(header, '')
            # Convert booleans to strings
            if isinstance(value, bool):
                value = 'true' if value else 'false'
            row.append(value)
        
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)
    
    def log_simple(self, test_id, command, mode, expected_intent, actual_intent,
                   context_tokens, inference_ms, success, notes=''):
        """
        Simple one-line logging for quick tests.
        
        Args:
            test_id: Test ID
            command: Voice command
            mode: 'filtered' or 'baseline'
            expected_intent: Expected intent
            actual_intent: Actual intent from LLM
            context_tokens: Token count
            inference_ms: Inference time
            success: Whether test passed
            notes: Optional notes
        """
        row = [
            datetime.now().strftime('%Y-%m-%d %H:%M:%S'),  # timestamp
            test_id,
            '',  # level
            command,
            mode,
            expected_intent,
            actual_intent,
            'true' if expected_intent == actual_intent else 'false',  # intent_correct
            '',  # expected_target
            '',  # actual_target
            '',  # target_correct
            context_tokens,
            inference_ms,
            '',  # planner_inference_ms
            '',  # plan_valid
            '',  # action_count
            'true' if success else 'false',  # execution_success
            '',  # failure_reason
            '',  # total_time_ms
            notes
        ]
        
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)


def create_logger(experiment_name='thesis_metrics'):
    """
    Create a metrics logger with a timestamped filename.
    
    Args:
        experiment_name: Base name for the CSV file
        
    Returns:
        MetricsLogger instance
    """
    # Create filename with timestamp
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f"{experiment_name}_{timestamp}.csv"
    
    # Put in home directory by default
    home_dir = os.path.expanduser('~')
    csv_path = os.path.join(home_dir, 'thesis_data', filename)
    
    print(f"Metrics will be logged to: {csv_path}")
    return MetricsLogger(csv_path)
