#!/bin/bash
# launch_swarm_test.sh
# ====================
# Launch 2-drone swarm test: Gazebo + Crazyswarm2 + Swarm Stack.
#
# Windows:
#   0: gazebo_sitl   — Gazebo server + 2 SITL firmware instances
#   1: gazebo_gui    — Gazebo GUI (visual)
#   2: crazyswarm2   — Crazyswarm2 cflib server (both drones)
#   3: swarm_stack   — action_server cf231 + cf232 + swarm_manager
#   4: commands      — ready for test commands
#
# Usage:
#   bash launch_swarm_test.sh
#   # (optional) run the goto test from window 4:
#   python3 ros_ws/thesis_tests/swarm_goto_test.py

set -e

SESSION_NAME="cocoa_swarm_test"
WORKSPACE=/home/infinity/Workspace/Cocoa/LLMAgent
ROS_WS=${WORKSPACE}/ros_ws
CF_FW_DIR=${WORKSPACE}/simulation_ws/CrazySim/crazyflie-firmware

SOURCE_CMD="source /opt/ros/humble/setup.bash && source ${ROS_WS}/install/setup.bash"

# Kill existing session
byobu kill-session -t ${SESSION_NAME} 2>/dev/null || true
sleep 1

echo "=================================================="
echo "  COCOA 2-Drone Swarm Test Launcher"
echo "=================================================="

byobu new-session -d -s ${SESSION_NAME}

# ── Window 0: Gazebo SITL (server + 2 firmware instances) ────────────────────
byobu rename-window -t ${SESSION_NAME}:0 "gazebo_sitl"
echo "[1/5] Starting Gazebo + 2 SITL instances..."
byobu send-keys -t ${SESSION_NAME}:0.0 \
    "bash ${WORKSPACE}/launch_sitl_two.sh" C-m

echo "      Waiting for Gazebo + drones to initialise (35s)..."
sleep 35

# ── Window 1: Gazebo GUI ──────────────────────────────────────────────────────
echo "[2/5] Starting Gazebo GUI..."
byobu new-window -t ${SESSION_NAME} -n "gazebo_gui"
sleep 1
byobu send-keys -t ${SESSION_NAME}:1.0 \
    "gz sim -g" C-m
sleep 5

# ── Window 2: Crazyswarm2 cflib server ───────────────────────────────────────
echo "[3/5] Starting Crazyswarm2 (cflib, 2 drones)..."
byobu new-window -t ${SESSION_NAME} -n "crazyswarm2"
sleep 1
byobu send-keys -t ${SESSION_NAME}:2.0 \
    "${SOURCE_CMD} && cd ${ROS_WS} && ros2 launch crazyflie launch.py backend:=cflib" C-m

echo "      Waiting for Crazyswarm2 to connect (15s)..."
sleep 15

# ── Window 3: Swarm stack (2x action servers + swarm manager) ────────────────
echo "[4/5] Starting Swarm stack..."
byobu new-window -t ${SESSION_NAME} -n "swarm_stack"
sleep 1
byobu send-keys -t ${SESSION_NAME}:3.0 \
    "${SOURCE_CMD} && cd ${ROS_WS} && ros2 launch cocoa_swarm swarm.launch.py" C-m

echo "      Waiting for swarm stack to start (8s)..."
sleep 8

# ── Window 4: Command window ──────────────────────────────────────────────────
echo "[5/5] Setting up command window..."
byobu new-window -t ${SESSION_NAME} -n "commands"
sleep 1
byobu send-keys -t ${SESSION_NAME}:4.0 "${SOURCE_CMD} && cd ${WORKSPACE}" C-m
sleep 1
byobu send-keys -t ${SESSION_NAME}:4.0 \
    "# Run swarm test:  python3 ros_ws/thesis_tests/swarm_goto_test.py" Enter
byobu send-keys -t ${SESSION_NAME}:4.0 \
    "# Custom target:   python3 ros_ws/thesis_tests/swarm_goto_test.py -5.0 3.0" Enter

# Go to first window
byobu select-window -t ${SESSION_NAME}:0

echo ""
echo "=================================================="
echo "  Swarm Test System Launched!"
echo "=================================================="
echo ""
echo "  0: gazebo_sitl   — Gazebo physics server + 2 SITL drones"
echo "  1: gazebo_gui    — Gazebo visual (see both drones)"
echo "  2: crazyswarm2   — ROS2 ↔ cflib bridge (cf231 + cf232)"
echo "  3: swarm_stack   — action servers + swarm_manager_node"
echo "  4: commands      — run your test here"
echo ""
echo "  ► To see the session, run in a new terminal:"
echo "    byobu attach-session -t ${SESSION_NAME}"
echo ""
echo "  ► From window 4 (commands), run:"
echo "    python3 ros_ws/thesis_tests/swarm_goto_test.py"
echo ""
echo "Byobu: F3/F4=prev/next window, F6=detach, F7=scroll"
echo ""
echo "All systems ready. Attach manually with:"
echo "  byobu attach-session -t ${SESSION_NAME}"
