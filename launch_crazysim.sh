#!/bin/bash

# CrazySim + Crazyswarm2 Launch Script using Byobu
# This script launches Gazebo SITL simulation and Crazyswarm2 cflib server

SESSION_NAME="crazysim"

# Configuration
CRAZYSIM_DIR="/home/ttz/LLMAgent-----Cocoa_Speech/simulation_ws/CrazySim/crazyflie-firmware"
CRAZYSWARM_WS="/home/ttz/LLMAgent-----Cocoa_Speech/ros_ws"
MODEL="crazyflie"
X_POS=0
Y_POS=0

# Kill existing session if it exists
byobu kill-session -t $SESSION_NAME 2>/dev/null
sleep 1

echo "=============================================="
echo "  CrazySim + Crazyswarm2 (Byobu)"
echo "=============================================="

# Create new byobu session
byobu new-session -d -s $SESSION_NAME

#############################################
# Window 0: Gazebo SITL
#############################################
byobu rename-window -t $SESSION_NAME:0 "gazebo_sitl"
echo "[1/2] Starting Gazebo SITL simulation..."
byobu send-keys -t $SESSION_NAME:0.0 "cd ${CRAZYSIM_DIR} && bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_singleagent.sh -m ${MODEL} -x ${X_POS} -y ${Y_POS}" C-m

# Wait for Gazebo to initialize
echo "      Waiting for Gazebo to initialize (20 seconds)..."
sleep 20

#############################################
# Window 1: Crazyswarm2 cflib server
#############################################
echo "[2/2] Starting Crazyswarm2 cflib server..."
byobu new-window -t $SESSION_NAME -n "crazyswarm2"
sleep 1
byobu send-keys -t $SESSION_NAME:1.0 "source /opt/ros/humble/setup.bash && source ${CRAZYSWARM_WS}/install/setup.bash && cd ${CRAZYSWARM_WS} && ros2 launch crazyflie launch.py backend:=cflib" C-m

# Go to first window
byobu select-window -t $SESSION_NAME:0

echo ""
echo "=============================================="
echo "  CrazySim launched in byobu session!"
echo "=============================================="
echo ""
echo "Windows (use F3/F4 to navigate):"
echo "  0: gazebo_sitl  - Gazebo SITL simulation"
echo "  1: crazyswarm2  - Crazyswarm2 cflib server"
echo ""
echo "Byobu Keys: F3/F4=prev/next window, F6=detach, F7=scroll"
echo ""
echo "Attaching to session..."

# Attach to the session
byobu attach-session -t $SESSION_NAME
