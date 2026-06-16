#!/bin/bash
# launch_swarm_nobyobu.sh
# Spawns everything in the background locally without Byobu so the Gazebo GUI easily opens.
set -e

WORKSPACE=/home/infinity/Workspace/Cocoa/LLMAgent
ROS_WS=${WORKSPACE}/ros_ws
SOURCE_CMD="source /opt/ros/humble/setup.bash && source ${ROS_WS}/install/setup.bash"

# 1. Kill everything
pkill -x cf2 || true
killall -9 gz || true
sleep 1

echo "[1/4] Starting Gazebo SITL..."
bash ${WORKSPACE}/launch_sitl_two.sh > /dev/null 2>&1 &
SITL_PID=$!

echo "Waiting 30s for SITL firmware instances to boot up..."
sleep 30

echo "[2/4] Opening Gazebo GUI..."
gz sim -g &
GUI_PID=$!
sleep 5

echo "[3/4] Starting Crazyswarm2 (Bridging ROS2 and the drones)..."
bash -c "${SOURCE_CMD} && cd ${ROS_WS} && ros2 launch crazyflie launch.py backend:=cflib" > /dev/null 2>&1 &
CFLIB_PID=$!

echo "Waiting 15s for Crazyswarm2..."
sleep 15

echo "[4/4] Starting cocoa_swarm action servers and manager..."
bash -c "${SOURCE_CMD} && cd ${ROS_WS} && ros2 launch cocoa_swarm swarm.launch.py" > /dev/null 2>&1 &
SWARM_PID=$!

echo ""
echo "============================================="
echo "   All systems successfully launched! 🚀     "
echo "============================================="
echo "You should now see the Gazebo GUI on your screen!"
echo ""
echo "To test the drone swarm movement, run the following in your terminal:"
echo "    cd /home/infinity/Workspace/Cocoa/LLMAgent/ros_ws"
echo "    source install/setup.bash"
echo "    python3 thesis_tests/swarm_goto_test.py"
echo "============================================="
echo ""
echo "Press [Ctrl+C] to stop all processes."

cleanup() {
    echo "Stopping background processes..."
    kill $SWARM_PID $CFLIB_PID $GUI_PID $SITL_PID 2>/dev/null
    pkill -x cf2 2>/dev/null || true
    killall -9 gz 2>/dev/null || true
    echo "Done."
}
trap cleanup EXIT

wait $SITL_PID
