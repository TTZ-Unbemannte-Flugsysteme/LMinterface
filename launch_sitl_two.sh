#!/bin/bash
# launch_sitl_two.sh
# ==================
# Launch Gazebo + TWO Crazyflie SITL instances:
#   cf231  N=0  ports: firmware=19950  cflib=19850  spawn @ x=0,  y=0
#   cf232  N=1  ports: firmware=19951  cflib=19851  spawn @ x=0,  y=-1.5
#
# Usage:
#   bash launch_sitl_two.sh [world_name]
#   world defaults to: cocoa_demo_warehouse

set -e

WORLD=${1:-cocoa_demo_warehouse}

CF_FW_DIR=/home/infinity/Workspace/Cocoa/LLMAgent/simulation_ws/CrazySim/crazyflie-firmware
BUILD_PATH=${CF_FW_DIR}/sitl_make/build
SRC_PATH=${CF_FW_DIR}
SETUP_SCRIPT=${SRC_PATH}/tools/crazyflie-simulation/simulator_files/gazebo/launch/setup_gz.bash
JINJA_SCRIPT=${SRC_PATH}/tools/crazyflie-simulation/simulator_files/gazebo/launch/jinja_gen.py
MODELS_DIR=${SRC_PATH}/tools/crazyflie-simulation/simulator_files/gazebo/models
WORLDS_DIR=${SRC_PATH}/tools/crazyflie-simulation/simulator_files/gazebo/worlds

echo "=== COCOA 2-Drone SITL Launcher ==="
echo "World: ${WORLD}"
echo "CF firmware dir: ${CF_FW_DIR}"

# ── Kill any running instances ────────────────────────────────────────────────
echo "[1/5] Killing existing firmware instances..."
pkill -x cf2 2>/dev/null || true
sleep 1

# ── Setup Gazebo environment ──────────────────────────────────────────────────
echo "[2/5] Setting up Gazebo environment..."
source ${SETUP_SCRIPT} ${SRC_PATH} ${BUILD_PATH}

export CF2_SIM_MODEL=gz_crazyflie

# ── spawn_model helper (mirrors sitl_singleagent.sh logic) ───────────────────
spawn_model() {
    local MODEL=$1
    local N=$2
    local X=$3
    local Y=$4

    local FIRM_PORT=$((19950 + N))
    local CFLIB_PORT=$((19850 + N))

    local WORK_DIR=${BUILD_PATH}/${N}
    mkdir -p "${WORK_DIR}"

    echo "[spawn_model] Generating SDF for ${MODEL}_${N} (firmware=${FIRM_PORT}, cflib=${CFLIB_PORT})"
    python3 ${JINJA_SCRIPT} \
        ${MODELS_DIR}/${MODEL}/model.sdf.jinja \
        ${SRC_PATH}/tools/crazyflie-simulation/simulator_files/gazebo \
        --cffirm_udp_port ${FIRM_PORT} \
        --cflib_udp_port  ${CFLIB_PORT} \
        --cf_id           ${N} \
        --cf_name         cf \
        --output-file     /tmp/${MODEL}_${N}.sdf

    echo "[spawn_model] Spawning ${MODEL}_${N} at (${X}, ${Y})"
    gz service \
        -s /world/${WORLD}/create \
        --reqtype  gz.msgs.EntityFactory \
        --reptype  gz.msgs.Boolean \
        --timeout  300 \
        --req "sdf_filename: \"/tmp/${MODEL}_${N}.sdf\", pose: {position: {x:${X}, y:${Y}, z:0.5}}, name: \"${MODEL}_${N}\", allow_renaming: 1"

    pushd "${WORK_DIR}" > /dev/null
    echo "[spawn_model] Starting firmware instance ${N} (port ${FIRM_PORT})"
    ${BUILD_PATH}/cf2 ${FIRM_PORT} > out.log 2> error.log &
    popd > /dev/null
}

# ── Start Gazebo (server + physics, no GUI) ───────────────────────────────────
echo "[3/5] Starting Gazebo server..."
gz sim -s -r ${WORLDS_DIR}/${WORLD}.sdf -v 3 &
GZ_PID=$!

echo "    Waiting 5s for Gazebo to initialise..."
sleep 5

# ── Spawn drone 1: cf231 (N=0) ───────────────────────────────────────────────
echo "[4/5] Spawning cf231 (N=0) @ (0, 0)..."
spawn_model crazyflie 0 0 0
sleep 3

# ── Spawn drone 2: cf232 (N=1) ───────────────────────────────────────────────
echo "[5/5] Spawning cf232 (N=1) @ (0, -1.5)..."
spawn_model crazyflie 1 0 -1.5
sleep 2

echo ""
echo "=== Both SITL instances running ==="
echo "  cf231: firmware port 19950, cflib port 19850"
echo "  cf232: firmware port 19951, cflib port 19851"
echo ""
echo "Next: ros2 launch crazyflie launch.py    (in ros_ws)"
echo "      ros2 launch cocoa_swarm swarm.launch.py"
echo ""
echo "Press Ctrl+C to stop all processes."

# ── Cleanup on exit ───────────────────────────────────────────────────────────
cleanup() {
    echo ""
    echo "[cleanup] Stopping firmware instances and Gazebo..."
    pkill -x cf2 2>/dev/null || true
    kill ${GZ_PID} 2>/dev/null || true
}
trap cleanup SIGINT SIGTERM EXIT

wait ${GZ_PID}
