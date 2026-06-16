# Thesis Test Procedure Guide
## LLM-Based Drone Control System

---

## Overview

Your thesis tests a **context-aware LLM for drone control**. The goal is to evaluate:
1. Can the LLM correctly interpret natural language commands?
2. Can the LLM use environmental context (EKG) for disambiguation?
3. What are the limitations of a 7B parameter model for this task?

---

## Phase 1: Automated Testing (30-40 minutes)

### Purpose
Collect **quantitative metrics**: pass/fail rates, inference time, token counts.

### Setup
```bash
# Terminal 1: Start simulation
cd ~/LLMAgent-----Cocoa_Speech/ros_ws
source install/setup.bash
ros2 launch cocoa_sim warehouse.launch.py  # or your simulation launch

# Terminal 2: Start LLM server
python -m llama_cpp.server --model <path_to_model> --n_gpu_layers 32 --n_ctx 4096

# Terminal 3: Start nodes
ros2 launch cocoa_intent intent_extractor.launch.py
ros2 launch cocoa_llm_planner planner.launch.py
ros2 launch cocoa_ekg ekg.launch.py
ros2 launch cocoa_lga lga.launch.py
```

### Run Tests
```bash
# Terminal 4: Run test sequences
cd ~/LLMAgent-----Cocoa_Speech/ros_ws/thesis_tests

# Run one sequence at a time
python3 run_batch_tests.py --mode filtered --seq 1
python3 run_batch_tests.py --mode filtered --seq 2
# ... continue for sequences 3-11

# Or run all at once (takes longer)
python3 run_batch_tests.py --mode filtered --seq all
```

### What to Record
- Pass/fail count per sequence
- Any timeouts or errors
- Save the CSV output files

---

## Phase 2: Manual Exploration (60-90 minutes)

### Purpose
Discover **qualitative findings** that automated tests miss.

### Setup
Same as Phase 1, but instead of running batch tests, manually publish commands:

```bash
# Publish a command manually
ros2 topic pub /voice/text std_msgs/msg/String "data: 'go to shelf A'" --once
```

### Exploration Categories

#### A. Ambiguous References (15 min)
Test how the LLM handles unclear commands:
```
"go to the shelf"           # Multiple shelves exist
"go to the pallet"          # Multiple pallets exist
"go there"                  # No clear reference
"go to that thing"          # Vague reference
"go near it"                # Unclear 'it'
```

#### B. Conversation History (15 min)
Test context from previous commands:
```
1. "go to shelf A"
2. "go to shelf B"
3. "go back to the other one"  # Should go to shelf_a
4. "do that again"             # Should repeat last action
5. "where was I before?"       # Query previous location
```

#### C. Edge Cases (15 min)
Test unusual inputs:
```
"move forward negative 5 meters"
"go to shelf_c"              # Non-existent object
"take off take off take off"  # Repeated command
"                     "       # Empty/whitespace
"do a backflip"              # Impossible action
"🚀 fly up"                   # Unicode/emoji
```

#### D. Complex Commands (15 min)
Multi-intent and complex phrasing:
```
"take off, go to shelf A, then come back here and land"
"visit all the pallets"
"patrol between shelf A and shelf B three times"
"go to the nearest pallet but avoid the forklift"
```

#### E. Linguistic Variations (15 min)
Same intent, different phrasing:
```
"go to shelf A" vs "navigate to shelf A" vs "move to shelf A"
"take off" vs "launch" vs "lift off" vs "fly up"
"land" vs "touch down" vs "come down" vs "return to ground"
```

### What to Record
For each test, record in `observations.md`:
1. Command given
2. LLM raw output (from terminal logs)
3. What actually happened
4. Classification: SUCCESS / PARTIAL / FAILURE
5. Notes on interesting behavior

---

## Phase 3: A/B Comparison (Optional, 30 min)

### Purpose
Compare LGA-filtered context vs baseline (full context).

### Run
```bash
# Filtered mode (LGA active)
python3 run_batch_tests.py --mode filtered --seq 1,2,3

# Baseline mode (full context)
python3 run_batch_tests.py --mode baseline --seq 1,2,3
```

### Compare
- Token counts (filtered should be lower)
- Inference time (filtered should be faster)
- Accuracy (may or may not differ)

---

## Phase 4: Document Findings (30 min)

### Compile Quantitative Results
From CSV files:
- Overall pass rate
- Pass rate per intent type
- Average inference time
- Token usage comparison (filtered vs baseline)

### Compile Qualitative Findings
From observations.md:
- Failure patterns (categorize why things fail)
- Model limitations (like the arithmetic issue you found)
- Surprising successes
- Recommendations for improvement

### Update findings.md
Add each significant finding with:
- Clear description of the issue
- Example input/output
- Root cause analysis
- Implications for the thesis

---

## Checklist

### Before Testing
- [ ] Simulation running and drone visible
- [ ] LLM server running (check with curl)
- [ ] All ROS nodes running (check with ros2 node list)
- [ ] Test files ready in thesis_tests/

### During Testing
- [ ] Save terminal logs (copy/paste interesting outputs)
- [ ] Note timestamps for significant events
- [ ] Screenshot any visual issues

### After Testing
- [ ] Collect all CSV result files
- [ ] Update findings.md with new discoveries
- [ ] Update observations.md with qualitative notes
- [ ] Backup all data

---

## Expected Findings Areas

Based on your tests so far, you'll likely find:

1. **Arithmetic Limitations** ✓ (Already documented)
2. **Context Window Limits** - Performance degradation with long history
3. **JSON Format Adherence** - LLM sometimes adds text outside JSON
4. **Object Name Hallucination** - LLM invents names like "forklift_1"
5. **Ambiguity Resolution** - How well it handles "the other one"
6. **Multi-Intent Sequencing** - Correct ordering of actions
7. **Safety Awareness** - Does it respect obstacles/battery warnings?
8. **Latency Distribution** - Variance in inference time

---

## Next Steps

1. Complete Phase 1 (automated tests)
2. Do Phase 2 (manual exploration) - this is where rich findings come from
3. Document everything in findings.md and observations.md
4. Analyze patterns and draw conclusions
