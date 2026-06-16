#!/bin/bash
# ============================================
# restart_and_fly.sh — Kill all, rebuild, relaunch, fly
# ============================================
# Usage:
#   ./restart_and_fly.sh                   # fly to shelf_a (default)
#   ./restart_and_fly.sh -10.0 8.0         # fly to specific coords
#   ./restart_and_fly.sh -10.0 8.0 6.0 2.0 # shelf_a then pallet_2
# ============================================

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="$SCRIPT_DIR/ros_ws"

# ── Step 1: Kill everything ──────────────────────────────────────────────────
echo "🔪 [1/5] Killing all processes..."
pkill -9 -f launch_swarm_nobyobu 2>/dev/null || true
pkill -9 -f swarm_manager_node 2>/dev/null || true
pkill -9 -f action_server_node 2>/dev/null || true
pkill -9 -f path_planner_node 2>/dev/null || true
pkill -9 -f crazyflie_server 2>/dev/null || true
pkill -9 -f "launch.py backend" 2>/dev/null || true
pkill -9 -f teleop 2>/dev/null || true
pkill -9 -f gui.py 2>/dev/null || true
pkill -9 -x cf2 2>/dev/null || true
killall -9 gz 2>/dev/null || true
sleep 3
# Double-check gz
for pid in $(pgrep -f "gz sim"); do kill -9 "$pid" 2>/dev/null; done
sleep 1
echo "   ✅ All clean"

# ── Step 2: Build ────────────────────────────────────────────────────────────
echo "🔨 [2/5] Building cocoa_swarm + cocoa_llm_planner..."
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash" 2>/dev/null || true
cd "$ROS_WS"
colcon build --symlink-install --packages-select cocoa_swarm cocoa_llm_planner 2>&1 | tail -3
source "$ROS_WS/install/setup.bash"
echo "   ✅ Build done"

# ── Step 3: Launch simulation ────────────────────────────────────────────────
echo "🚀 [3/5] Launching Gazebo + Crazyswarm2 + Swarm stack..."
cd "$SCRIPT_DIR"
./launch_swarm_nobyobu.sh > /tmp/launch_swarm_latest.log 2>&1 &
LAUNCH_PID=$!

# Wait for all nodes to be ready
echo "   Waiting for SITL firmware (30s)..."
sleep 30
echo "   Waiting for Crazyswarm2 (15s)..."
sleep 15
echo "   Waiting for swarm nodes (15s)..."
sleep 15

# Verify
CF2_COUNT=$(pgrep -c -x cf2 2>/dev/null || echo 0)
SWARM_COUNT=$(pgrep -c -f swarm_manager_node 2>/dev/null || echo 0)
PATH_COUNT=$(pgrep -c -f path_planner_node 2>/dev/null || echo 0)
ACT_COUNT=$(pgrep -c -f action_server_node 2>/dev/null || echo 0)

echo "   cf2=$CF2_COUNT  swarm=$SWARM_COUNT  path_planner=$PATH_COUNT  action_servers=$ACT_COUNT"

if [ "$SWARM_COUNT" -eq 0 ] || [ "$ACT_COUNT" -eq 0 ]; then
    echo "   ❌ Some nodes failed to start. Check /tmp/launch_swarm_latest.log"
    exit 1
fi
echo "   ✅ All nodes running"

# ── Step 4: Unpause simulation ───────────────────────────────────────────────
echo "▶️  [4/5] Unpausing Gazebo..."
gz service -s /world/cocoa_demo_warehouse/control \
    --reqtype gz.msgs.WorldControl \
    --reptype gz.msgs.Boolean \
    --timeout 2000 \
    --req "pause: false" > /dev/null 2>&1
echo "   ✅ Simulation running"

# ── Step 5: Fly! ─────────────────────────────────────────────────────────────
cd "$ROS_WS"
source install/setup.bash

# Parse args in pairs (x y)
TARGETS=("$@")
if [ ${#TARGETS[@]} -eq 0 ]; then
    # Default: fly to shelf_a
    TARGETS=(-10.0 8.0)
fi

FLIGHT=1
i=0
while [ $i -lt ${#TARGETS[@]} ]; do
    TX="${TARGETS[$i]}"
    TY="${TARGETS[$((i+1))]}"
    echo ""
    echo "✈️  [5/5] Flight $FLIGHT: target ($TX, $TY)"
    echo "============================================"
    python3 thesis_tests/swarm_goto_test.py "$TX" "$TY"
    FLIGHT=$((FLIGHT + 1))
    i=$((i + 2))
done

echo ""
echo "============================================"
echo "  All flights complete! 🎯"
echo "  To stop: kill $LAUNCH_PID"
echo "============================================"
