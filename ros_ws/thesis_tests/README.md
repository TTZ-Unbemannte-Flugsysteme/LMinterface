# Thesis Test Scripts

This folder contains all test scripts for the thesis experiments.

## Quick Start

```bash
# Source ROS2
source /opt/ros/humble/setup.bash
source ~/LLMAgent-----Cocoa_Speech/ros_ws/install/setup.bash

# Run a single test to verify system works
python3 single_test.py "take off"

# Run Level 1 tests in filtered mode
python3 run_batch_tests.py --mode filtered --level 1

# Run all tests in baseline mode
python3 run_batch_tests.py --mode baseline --all
```

## Files

| File | Purpose |
|------|---------|
| `single_test.py` | Test single command - use to verify system |
| `run_batch_tests.py` | Run multiple tests, log to CSV |
| `test_cases.py` | All 100 test cases organized by level |
| `metrics_logger.py` | CSV logging utility |

## Test Levels

- **Level 1**: Single-step commands (take off, land, move)
- **Level 2**: Object-referenced (go to shelf A)
- **Level 3**: Context-dependent (conversation history)
- **Level 4**: Multi-step commands (take off and go to shelf A)

## Usage Examples

```bash
# Quick system check
python3 single_test.py "take off"
python3 single_test.py "go to shelf A"

# Run Level 2 in filtered mode
python3 run_batch_tests.py --mode filtered --level 2

# Run all tests with custom output
python3 run_batch_tests.py --mode baseline --all --output my_results.csv
```

## CSV Output

Results are saved to CSV with columns:
- test_id, level, command, mode
- expected_intent, actual_intent, intent_correct
- context_tokens, inference times
- execution_success, failure_reason
