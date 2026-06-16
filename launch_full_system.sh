#!/bin/bash
# Full System Launch Script using Byobu
# Combines: Simulation (Gazebo SITL + Crazyswarm2) + Cocoa ROS2 Nodes
# Order: Simulation first, then ROS nodes
#
# Usage: ./launch_full_system.sh

SESSION_NAME="cocoa_full"

# Configuration
ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
EKG_CONFIG=${1:-"objects.yaml"}
CRAZYSIM_DIR="${ROOT_DIR}/simulation_ws/CrazySim/crazyflie-firmware"
CRAZYSWARM_WS="${ROOT_DIR}/ros_ws"
MODEL_PATH="${ROOT_DIR}/ros_ws/edge_models/qwen2.5-7b-instruct-q5_0-00001-of-00002.gguf"
MODEL="crazyflie"
WORLD="cocoa_demo_warehouse"
X_POS=0
Y_POS=0


# OpenAI/Whisper API Key (Required if using OpenAI/Whisper cloud API)
# export OPENAI_API_KEY="your-api-key-here"

# Source ROS2 workspace command
SOURCE_CMD="export OPENAI_API_KEY='$OPENAI_API_KEY' && source /opt/ros/humble/setup.bash && source ${CRAZYSWARM_WS}/install/setup.bash"

# Kill existing session if it exists
byobu kill-session -t $SESSION_NAME 2>/dev/null
sleep 1

echo "=================================================="
echo "  Full System Launch: Simulation + Cocoa ROS2"
echo "=================================================="

# Create new byobu session
byobu new-session -d -s $SESSION_NAME

############################################################
# PART 1: SIMULATION LAYER
############################################################
echo ""
echo "--- SIMULATION LAYER ---"

#############################################
# Window 0: Gazebo SITL
#############################################
byobu rename-window -t $SESSION_NAME:0 "gazebo_sitl"
echo "[1/11] Starting Gazebo SITL simulation..."
byobu send-keys -t $SESSION_NAME:0.0 "cd ${CRAZYSIM_DIR} && bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_singleagent.sh -m ${MODEL} -x ${X_POS} -y ${Y_POS} -w ${WORLD}" C-m

# Wait for Gazebo to initialize
echo "       Waiting for Gazebo to initialize (20 seconds)..."
sleep 20

#############################################
# Window 1: Crazyswarm2 cflib server
#############################################
echo "[2/11] Starting Crazyswarm2 cflib server..."
byobu new-window -t $SESSION_NAME -n "crazyswarm2"
sleep 1
byobu send-keys -t $SESSION_NAME:1.0 "$SOURCE_CMD && cd ${CRAZYSWARM_WS} && ros2 launch crazyflie launch.py backend:=cflib" C-m

# Wait for Crazyswarm2 to connect to simulation
echo "       Waiting for Crazyswarm2 to connect (10 seconds)..."
sleep 10

############################################################
# PART 2: ROS2 COCOA NODES
############################################################
echo ""
echo "--- ROS2 COCOA NODES ---"

#############################################
# Window 2: LLM Server
#############################################
byobu new-window -t $SESSION_NAME -n "llm_server"
echo "[3/11] Starting Llama.cpp Server..."
byobu send-keys -t $SESSION_NAME:2.0 "python -m llama_cpp.server --model ${MODEL_PATH} --n_gpu_layers 32 --n_ctx 8192 --host 0.0.0.0 --port 8081" C-m
sleep 5

#############################################
# Window 3: EKG Node
#############################################
echo "[4/11] Starting EKG Node with ${EKG_CONFIG}..."
byobu new-window -t $SESSION_NAME -n "ekg"
sleep 1
byobu send-keys -t $SESSION_NAME:3.0 "$SOURCE_CMD && ros2 run cocoa_ekg ekg_node --ros-args -p config_file:=${EKG_CONFIG}" C-m
sleep 5  # Give EKG time to initialize services

#############################################
# Window 4: LGA Node
#############################################
echo "[5/11] Starting LGA Node..."
byobu new-window -t $SESSION_NAME -n "lga"
sleep 1
byobu send-keys -t $SESSION_NAME:4.0 "$SOURCE_CMD && ros2 run cocoa_lga lga_node" C-m
sleep 2

#############################################
# Window 5: Intent Launch
#############################################
echo "[6/11] Starting Intent Launch..."
byobu new-window -t $SESSION_NAME -n "intent"
sleep 1
byobu send-keys -t $SESSION_NAME:5.0 "$SOURCE_CMD && ros2 launch cocoa_intent intent.launch.py" C-m
sleep 2

#############################################
# Window 6: Action Server Node
#############################################
echo "[7/11] Starting Action Server Node..."
byobu new-window -t $SESSION_NAME -n "action_srv"
sleep 1
byobu send-keys -t $SESSION_NAME:6.0 "$SOURCE_CMD && ros2 run cocoa_llm_planner action_server_node" C-m
sleep 2

#############################################
# Window 7: LLM Planner Node
#############################################
echo "[8/11] Starting LLM Planner Node..."
byobu new-window -t $SESSION_NAME -n "llm_planner"
sleep 1
byobu send-keys -t $SESSION_NAME:7.0 "$SOURCE_CMD && ros2 run cocoa_llm_planner llm_planner_node" C-m
sleep 2

#############################################
# Window 8: Path Planner Node
#############################################
echo "[9/11] Starting Path Planner Node..."
byobu new-window -t $SESSION_NAME -n "path_planner"
sleep 1
byobu send-keys -t $SESSION_NAME:8.0 "$SOURCE_CMD && ros2 run cocoa_path_planner path_planner_node" C-m
sleep 2

#############################################
# Window 9: Web Interface Node
#############################################
echo "[10/11] Starting Web Interface Node..."
byobu new-window -t $SESSION_NAME -n "web_ui"
sleep 1
byobu send-keys -t $SESSION_NAME:9.0 "$SOURCE_CMD && ros2 run cocoa_dialogue web_interface_node" C-m
sleep 2

#############################################
# Window 10: Command window for testing
#############################################
echo "[11/11] Setting up command window..."
byobu new-window -t $SESSION_NAME -n "commands"
sleep 1
byobu send-keys -t $SESSION_NAME:10.0 "$SOURCE_CMD" C-m
sleep 0.5
byobu send-keys -t $SESSION_NAME:10.0 "# Ready for test commands!" Enter
byobu send-keys -t $SESSION_NAME:10.0 "# ros2 topic pub /voice/text std_msgs/msg/String \"data: 'can you go to shelf_a'\" --once" Enter

# Go to first window (Gazebo)
byobu select-window -t $SESSION_NAME:0

echo ""
echo "=================================================="
echo "  Full System Launched!"
echo "=================================================="
echo ""
echo "Windows (use F3/F4 to navigate):"
echo ""
echo "  --- SIMULATION ---"
echo "  0: gazebo_sitl   - Gazebo SITL simulation"
echo "  1: crazyswarm2   - Crazyswarm2 cflib server"
echo ""
echo "  --- ROS2 NODES ---"
echo "  2: llm_server    - Llama.cpp server"
echo "  3: ekg           - EKG Node"
echo "  4: lga           - LGA Node"
echo "  5: intent        - Intent Launch"
echo "  6: action_srv    - Action Server Node"
echo "  7: llm_planner   - LLM Planner Node"
echo "  8: path_planner  - Path Planner Node"
echo "  9: web_ui        - Web Interface (http://localhost:5000)"
echo " 10: commands      - Test commands"
echo ""
echo "Byobu Keys: F3/F4=prev/next window, F6=detach, F7=scroll"
echo ""
echo "Attaching to session..."

# Attach to the session
byobu attach-session -t $SESSION_NAME
