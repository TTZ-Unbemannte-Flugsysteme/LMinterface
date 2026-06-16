"""
Thesis Test Cases - State-Tracking Sequences

Each test explicitly tracks:
- pre_state: Expected drone state BEFORE this command
- post_state: Expected drone state AFTER this command (if successful)
- expected_outcome: SUCCESS or REJECTION

States: 'grounded', 'flying'
"""

# ============================================================================
# SEQUENCE 1: Basic Flight Cycle
# Purpose: Test takeoff, basic navigation, and landing
# ============================================================================

SEQUENCE_1 = [
    # Start grounded
    {'id': 'S1-01', 'command': 'Take off',
     'pre_state': 'grounded', 'post_state': 'flying',
     'expected_intent': 'TAKEOFF', 'expected_outcome': 'SUCCESS'},
    
    # Flying - navigation commands
    {'id': 'S1-02', 'command': 'Go to shelf A',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION', 'expected_target': 'shelf_a'},
    
    {'id': 'S1-03', 'command': 'Move forward 2 meters',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'MOVE_DIRECTION'},
    
    {'id': 'S1-04', 'command': 'Go up 1 meter',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'CHANGE_ALTITUDE'},
    
    {'id': 'S1-05', 'command': 'Turn left',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'ROTATE'},
    
    {'id': 'S1-06', 'command': 'Land',
     'pre_state': 'flying', 'post_state': 'grounded',
     'expected_intent': 'LAND'},
    
    # Now grounded - test rejection
    {'id': 'S1-07', 'command': 'Land now',
     'pre_state': 'grounded', 'post_state': 'grounded',
     'expected_intent': 'LAND', 'expected_outcome': 'REJECTION',
     'notes': 'Correct behavior: reject landing when already grounded'},
]


# ============================================================================
# SEQUENCE 2: Navigation and Coreference
# Purpose: Test GO_TO_LOCATION and context/history usage
# ============================================================================

SEQUENCE_2 = [
    {'id': 'S2-01', 'command': 'Take off',
     'pre_state': 'grounded', 'post_state': 'flying',
     'expected_intent': 'TAKEOFF'},
    
    {'id': 'S2-02', 'command': 'Go to shelf A',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION', 'expected_target': 'shelf_a'},
    
    {'id': 'S2-03', 'command': 'Go to shelf B',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION', 'expected_target': 'shelf_b'},
    
    # Coreference test - "the other shelf"
    {'id': 'S2-04', 'command': 'Go back to the other shelf',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION', 'expected_target': 'shelf_a',
     'notes': 'Tests coreference: "other shelf" = shelf_a (came from shelf_b)'},
    
    {'id': 'S2-05', 'command': 'Land',
     'pre_state': 'flying', 'post_state': 'grounded',
     'expected_intent': 'LAND'},
]


# ============================================================================
# SEQUENCE 3: Queries and Status
# Purpose: Test QUERY intents (work in any state)
# ============================================================================

SEQUENCE_3 = [
    # Queries while grounded
    {'id': 'S3-01', 'command': "What's my battery?",
     'pre_state': 'grounded', 'post_state': 'grounded',
     'expected_intent': 'QUERY'},
    
    {'id': 'S3-02', 'command': 'Where am I?',
     'pre_state': 'grounded', 'post_state': 'grounded',
     'expected_intent': 'QUERY'},
    
    {'id': 'S3-03', 'command': 'Take off',
     'pre_state': 'grounded', 'post_state': 'flying',
     'expected_intent': 'TAKEOFF'},
    
    # Queries while flying
    {'id': 'S3-04', 'command': 'How high am I?',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'QUERY'},
    
    {'id': 'S3-05', 'command': 'Status report',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'QUERY'},
    
    {'id': 'S3-06', 'command': 'Land',
     'pre_state': 'flying', 'post_state': 'grounded',
     'expected_intent': 'LAND'},
]


# ============================================================================
# SEQUENCE 4: Movement Commands
# Purpose: Test MOVE_DIRECTION and CHANGE_ALTITUDE
# ============================================================================

SEQUENCE_4 = [
    {'id': 'S4-01', 'command': 'Take off',
     'pre_state': 'grounded', 'post_state': 'flying',
     'expected_intent': 'TAKEOFF'},
    
    {'id': 'S4-02', 'command': 'Move forward 1 meter',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'MOVE_DIRECTION'},
    
    {'id': 'S4-03', 'command': 'Go backward 1.5 meters',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'MOVE_DIRECTION'},
    
    {'id': 'S4-04', 'command': 'Move left 2 meters',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'MOVE_DIRECTION'},
    
    {'id': 'S4-05', 'command': 'Go right 1 meter',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'MOVE_DIRECTION'},
    
    {'id': 'S4-06', 'command': 'Rise 2 meters',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'CHANGE_ALTITUDE'},
    
    {'id': 'S4-07', 'command': 'Descend 1 meter',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'CHANGE_ALTITUDE'},
    
    {'id': 'S4-08', 'command': 'Land',
     'pre_state': 'flying', 'post_state': 'grounded',
     'expected_intent': 'LAND'},
]


# ============================================================================
# SEQUENCE 5: Rotation and Hover
# Purpose: Test ROTATE and HOVER intents
# ============================================================================

SEQUENCE_5 = [
    {'id': 'S5-01', 'command': 'Take off',
     'pre_state': 'grounded', 'post_state': 'flying',
     'expected_intent': 'TAKEOFF'},
    
    {'id': 'S5-02', 'command': 'Turn left',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'ROTATE'},
    
    {'id': 'S5-03', 'command': 'Rotate right 90 degrees',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'ROTATE'},
    
    {'id': 'S5-04', 'command': 'Turn around',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'ROTATE'},
    
    {'id': 'S5-05', 'command': 'Hover',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'HOVER'},
    
    {'id': 'S5-06', 'command': 'Stay here',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'HOVER'},
    
    {'id': 'S5-07', 'command': 'Wait',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'HOVER'},
    
    {'id': 'S5-08', 'command': 'Land',
     'pre_state': 'flying', 'post_state': 'grounded',
     'expected_intent': 'LAND'},
]


# ============================================================================
# SEQUENCE 6: Multi-Location Navigation
# Purpose: Test multiple GO_TO_LOCATION commands
# ============================================================================

SEQUENCE_6 = [
    {'id': 'S6-01', 'command': 'Take off',
     'pre_state': 'grounded', 'post_state': 'flying',
     'expected_intent': 'TAKEOFF'},
    
    {'id': 'S6-02', 'command': 'Go to pallet 1',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION', 'expected_target': 'pallet_1'},
    
    {'id': 'S6-03', 'command': 'Fly to pallet 2',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION', 'expected_target': 'pallet_2'},
    
    {'id': 'S6-04', 'command': 'Navigate to the forklift',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION', 'expected_target': 'forklift'},
    
    {'id': 'S6-05', 'command': 'Go to the landing pad',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION', 'expected_target': 'landing_pad'},
    
    {'id': 'S6-06', 'command': 'Land',
     'pre_state': 'flying', 'post_state': 'grounded',
     'expected_intent': 'LAND'},
]


# ============================================================================
# SEQUENCE 7: State-Aware Rejections
# Purpose: Test that LLM correctly rejects invalid commands for current state
# ============================================================================

SEQUENCE_7 = [
    # Try takeoff when grounded - should work
    {'id': 'S7-01', 'command': 'Take off',
     'pre_state': 'grounded', 'post_state': 'flying',
     'expected_intent': 'TAKEOFF'},
    
    # Try takeoff again when flying - should reject
    {'id': 'S7-02', 'command': 'Take off',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'TAKEOFF', 'expected_outcome': 'REJECTION',
     'notes': 'Correct: reject takeoff when already flying'},
    
    {'id': 'S7-03', 'command': 'Land',
     'pre_state': 'flying', 'post_state': 'grounded',
     'expected_intent': 'LAND'},
    
    # Try land again when grounded - should reject
    {'id': 'S7-04', 'command': 'Land',
     'pre_state': 'grounded', 'post_state': 'grounded',
     'expected_intent': 'LAND', 'expected_outcome': 'REJECTION',
     'notes': 'Correct: reject land when already grounded'},
]


# ============================================================================
# SEQUENCE 8: Multi-Intent Commands
# Purpose: Test compound commands like "take off and go to shelf A"
# ============================================================================

SEQUENCE_8 = [
    # Multi-intent from ground
    {'id': 'S8-01', 'command': 'Take off and go to shelf A',
     'pre_state': 'grounded', 'post_state': 'flying',
     'expected_intent': 'TAKEOFF',  # First intent
     'expected_action_count': 2,    # TAKEOFF + GO_TO_LOCATION
     'notes': 'Multi-intent: TAKEOFF + GO_TO_LOCATION'},
    
    # Multi-intent while flying
    {'id': 'S8-02', 'command': 'Go to shelf B then land',
     'pre_state': 'flying', 'post_state': 'grounded',
     'expected_intent': 'GO_TO_LOCATION',  # First intent
     'expected_action_count': 2,           # GO_TO_LOCATION + LAND
     'notes': 'Multi-intent: GO_TO_LOCATION + LAND'},
]


# ============================================================================
# SEQUENCE 9: EKG Grounding - Generic to Specific Object Resolution
# Purpose: Test LLM's ability to ground generic references to EKG objects
# ============================================================================

SEQUENCE_9 = [
    {'id': 'S9-01', 'command': 'Take off',
     'pre_state': 'grounded', 'post_state': 'flying',
     'expected_intent': 'TAKEOFF'},
    
    # Generic object references - LLM must resolve to EKG objects
    {'id': 'S9-02', 'command': 'Go to a shelf',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION',
     'notes': 'LLM should pick shelf_a or shelf_b from EKG'},
    
    {'id': 'S9-03', 'command': 'Go to a pallet',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION',
     'notes': 'LLM should pick pallet_1 or pallet_2 from EKG'},
    
    # Nearest object reasoning
    {'id': 'S9-04', 'command': 'Go to the nearest shelf',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION',
     'notes': 'LLM should pick closest shelf based on drone position'},
    
    # Relationship-based grounding
    {'id': 'S9-05', 'command': 'Go to the pallet near shelf A',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION', 'expected_target': 'pallet_1',
     'notes': 'Tests EKG relationship: pallet_1 is near shelf_a'},
    
    {'id': 'S9-06', 'command': 'Go to the pallet near shelf B',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION', 'expected_target': 'pallet_2',
     'notes': 'Tests EKG relationship: pallet_2 is near shelf_b'},
    
    {'id': 'S9-07', 'command': 'Land',
     'pre_state': 'flying', 'post_state': 'grounded',
     'expected_intent': 'LAND'},
]


# ============================================================================
# SEQUENCE 10: Conversation History Interpretation
# Purpose: Test LLM's ability to use past conversation for disambiguation
# ============================================================================

SEQUENCE_10 = [
    {'id': 'S10-01', 'command': 'Take off',
     'pre_state': 'grounded', 'post_state': 'flying',
     'expected_intent': 'TAKEOFF'},
    
    {'id': 'S10-02', 'command': 'Go to shelf A',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION', 'expected_target': 'shelf_a'},
    
    # "There" refers to previous location
    {'id': 'S10-03', 'command': 'Go to pallet 1',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION', 'expected_target': 'pallet_1'},
    
    {'id': 'S10-04', 'command': 'Go back ',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION', 'expected_target': 'shelf_a',
     'notes': 'Tests history: "previous location" = shelf_a (previous location)'},
    
    # "That" refers to last mentioned object
    {'id': 'S10-05', 'command': 'Go to shelf B',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION', 'expected_target': 'shelf_b'},
    
    {'id': 'S10-06', 'command': 'Go to the other one',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION', 'expected_target': 'shelf_a',
     'notes': 'Tests history: "other one" = shelf_a (not shelf_b)'},
    
    # "Again" refers to last action
    {'id': 'S10-07', 'command': 'Move forward',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'MOVE_DIRECTION'},
    
    {'id': 'S10-08', 'command': 'Do that again',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'MOVE_DIRECTION',
     'notes': 'Tests history: "do that again" = repeat forward movement'},
    
    {'id': 'S10-09', 'command': 'Land',
     'pre_state': 'flying', 'post_state': 'grounded',
     'expected_intent': 'LAND'},
]


# ============================================================================
# SEQUENCE 11: Multi-Destination Commands
# Purpose: Test commands with multiple locations in sequence
# ============================================================================

SEQUENCE_11 = [
    {'id': 'S11-01', 'command': 'Take off',
     'pre_state': 'grounded', 'post_state': 'flying',
     'expected_intent': 'TAKEOFF'},
    
    # Two destinations - should produce 2 GO_TO_LOCATION intents
    {'id': 'S11-02', 'command': 'Go to shelf A then go to shelf B',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION',
     'expected_targets': ['shelf_a', 'shelf_b'],
     'expected_intent_count': 2,
     'notes': 'Multi-destination: should produce 2 GO_TO_LOCATION intents'},
    
    # Three destinations - should produce 3 GO_TO_LOCATION intents
    {'id': 'S11-03', 'command': 'Visit pallet 1, pallet 2, and the forklift',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION',
     'expected_targets': ['pallet_1', 'pallet_2', 'forklift'],
     'expected_intent_count': 3,
     'notes': 'Multi-destination: should produce 3 GO_TO_LOCATION intents'},
    
    # Tour/patrol command - should visit all shelves
    {'id': 'S11-04', 'command': 'Patrol all the shelves',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION',
     'expected_targets': ['shelf_a', 'shelf_b'],
     'expected_intent_count': 2,
     'notes': 'Should produce 2 GO_TO_LOCATION intents for both shelves'},
    
    {'id': 'S11-05', 'command': 'Land',
     'pre_state': 'flying', 'post_state': 'grounded',
     'expected_intent': 'LAND'},
]


# ============================================================================
# SEQUENCE 12: Corner Cases and Edge Conditions
# Purpose: Test unusual/ambiguous/malformed inputs
# ============================================================================

SEQUENCE_12 = [
    {'id': 'S12-01', 'command': 'Take off',
     'pre_state': 'grounded', 'post_state': 'flying',
     'expected_intent': 'TAKEOFF'},
    
    # Unknown object - should handle gracefully
    {'id': 'S12-02', 'command': 'Go to the coffee machine',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION', 'expected_outcome': 'REJECTION',
     'notes': 'No coffee machine in EKG - should reject or ask for clarification'},
    
    # Typo/misspelling
    {'id': 'S12-03', 'command': 'Go to sheld A',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION', 'expected_target': 'shelf_a',
     'notes': 'Tests robustness: "sheld" = "shelf" (typo correction)'},
    
    # Very long distance
    {'id': 'S12-04', 'command': 'Move forward 100 meters',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'MOVE_DIRECTION',
     'notes': 'Large distance - LLM or planner may limit this'},
    
    # Zero distance
    {'id': 'S12-05', 'command': 'Move forward 0 meters',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'MOVE_DIRECTION',
     'notes': 'Zero movement - may be rejected or treated as hover'},
    
    # Negative distance
    {'id': 'S12-06', 'command': 'Move forward minus 2 meters',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'MOVE_DIRECTION',
     'notes': 'Negative value - should interpret as backward or reject'},
    
    # Contradictory command
    {'id': 'S12-07', 'command': 'Take off and land',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'TAKEOFF',
     'notes': 'Contradictory - may reject takeoff (already flying) then land'},
    
    # Empty/vague command
    {'id': 'S12-08', 'command': 'Go somewhere',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION', 'expected_outcome': 'REJECTION',
     'notes': 'Too vague - should ask for clarification'},
    
    # Polite/natural phrasing
    {'id': 'S12-09', 'command': 'Could you please fly over to shelf A?',
     'pre_state': 'flying', 'post_state': 'flying',
     'expected_intent': 'GO_TO_LOCATION', 'expected_target': 'shelf_a',
     'notes': 'Natural language with politeness markers'},
    
    {'id': 'S12-10', 'command': 'Land',
     'pre_state': 'flying', 'post_state': 'grounded',
     'expected_intent': 'LAND'},
]


# ============================================================================
# ALL SEQUENCES
# ============================================================================

ALL_SEQUENCES = {
    1: SEQUENCE_1,
    2: SEQUENCE_2,
    3: SEQUENCE_3,
    4: SEQUENCE_4,
    5: SEQUENCE_5,
    6: SEQUENCE_6,
    7: SEQUENCE_7,
    8: SEQUENCE_8,
    9: SEQUENCE_9,
    10: SEQUENCE_10,
    11: SEQUENCE_11,
    12: SEQUENCE_12,
}

ALL_TESTS = (SEQUENCE_1 + SEQUENCE_2 + SEQUENCE_3 + SEQUENCE_4 + 
             SEQUENCE_5 + SEQUENCE_6 + SEQUENCE_7 + SEQUENCE_8 +
             SEQUENCE_9 + SEQUENCE_10 + SEQUENCE_11 + SEQUENCE_12)


def get_sequence(seq_num):
    """Get a specific sequence by number."""
    return ALL_SEQUENCES.get(seq_num, [])


def get_all_tests():
    """Get all tests from all sequences."""
    return ALL_TESTS


def get_tests_by_category(category):
    """Get tests by category name."""
    if category == 'all':
        return ALL_TESTS
    elif category.startswith('seq'):
        num = int(category.replace('seq', ''))
        return get_sequence(num)
    return ALL_TESTS


# Summary
if __name__ == '__main__':
    print("State-Tracking Test Sequences Summary")
    print("=" * 50)
    for seq_num, tests in ALL_SEQUENCES.items():
        print(f"Sequence {seq_num}: {len(tests)} tests")
        grounded = sum(1 for t in tests if t['pre_state'] == 'grounded')
        flying = sum(1 for t in tests if t['pre_state'] == 'flying')
        rejections = sum(1 for t in tests if t.get('expected_outcome') == 'REJECTION')
        print(f"  - From grounded: {grounded}, From flying: {flying}")
        if rejections:
            print(f"  - Expected rejections: {rejections}")
    print(f"\nTotal tests: {len(ALL_TESTS)}")

