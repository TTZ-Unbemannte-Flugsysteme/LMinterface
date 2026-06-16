# Cocoa (Compressed Observation and Context Abstraction) - UAV control using Voice User Interface & Spatial Anchoring using Environment Knowledge Graph

A ROS2-based project that enables intelligent control for the Crazyflie nano quadcopter using Large Language Models (LLM).

## Overview

This project integrates LLM-based planning with the Crazyflie drone platform, allowing users to control the drone using natural language commands. It combines ROS2 (Robot Operating System 2) with Gazebo simulation via CrazySim's Software-in-the-Loop (SITL) backend.

## Components

The custom software interface consists of several key ROS2 packages (located under `ros_ws/src/`):

*   **cocoa_dialogue**: Web-based interface for interaction, command input, and speech-to-text.
*   **cocoa_ekg**: Epistemic Knowledge Graph (EKG) for managing world state and objects.
*   **cocoa_intent**: Intent extraction node that processes natural language commands.
*   **cocoa_lga**: Language-Guided Abstraction (LGA) node for environmental grounding.
*   **cocoa_llm_planner**: LLM-based action planner that executes high-level goals.
*   **cocoa_msgs**: Custom ROS2 message and service definitions.
*   **cocoa_path_planner**: A* path planner for collision-free navigation logic.
*   **cocoa_swarm**: Swarm manager node.

### Simulation & Drivers Included:
*   **crazyswarm2**: ROS2 driver for Crazyflie.
*   **ros_gz_crazyflie**: Gazebo-ROS2 bridge packages.
*   **CrazySim** (in `simulation_ws/`): Software-in-the-Loop simulator for Crazyflie.

---

## Prerequisites

| Requirement | Version | Notes |
|---|---|---|
| **Ubuntu** | 22.04 (Jammy) | Only tested platform |
| **ROS2** | Humble | Desktop install recommended |
| **Gazebo** | Garden (v7.x) | **Not Harmonic** — CrazySim requires Garden |
| **Python** | 3.10 | Ships with Ubuntu 22.04 |

---

## Installation

### Step 1: Clone the Repository

```bash
git clone https://github.com/TTZ-Unbemannte-Flugsysteme/LMinterface.git
cd LMinterface
```

### Step 2: Apply Custom Simulation Patch

Apply the patch to the CrazySim simulation tools:
```bash
cd simulation_ws/CrazySim/crazyflie-firmware/tools/crazyflie-simulation
patch -p1 < ../../../../../my_custom_fix.patch
cd ../../../../../
```

### Step 3: Install CrazySim's Patched cflib

```bash
cd simulation_ws/CrazySim/crazyflie-lib-python
pip install -e .
cd ../../..
```

### Step 4: Configure Simulation Assets

Link the custom world files:
```bash
ln -sf $(pwd)/my_sim_assets/*.sdf \
  $(pwd)/simulation_ws/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/worlds/
```

### Step 5: Build CrazySim (Firmware + Gazebo Plugin)

```bash
cd simulation_ws/CrazySim/crazyflie-firmware
mkdir -p sitl_make/build && cd sitl_make/build
cmake ..
make all
cd ../../../../..
```

---

## Usage

### Set Up Environment Paths

Add the following to your `~/.bashrc` (adjust `ROOT` to your clone location):

```bash
source /opt/ros/humble/setup.bash
ROOT="$HOME/path/to/LMinterface"
source "${ROOT}/ros_ws/install/setup.bash"

# Gazebo resource paths
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${ROOT}/my_sim_assets"

# OpenAI/Whisper API Key (Required if using OpenAI/Whisper cloud API)
export OPENAI_API_KEY="your-api-key-here"
```

### Install Remaining Dependencies

```bash
sudo apt install -y \
  ros-humble-tf-transformations \
  ros-humble-motion-capture-tracking \
  ros-humble-motion-capture-tracking-interfaces \
  byobu

pip install rowan nicegui==1.4.36 sse-starlette==1.8.2 numpy==1.23.5 faster-whisper
```

### Build the ROS2 Workspace

```bash
source /opt/ros/humble/setup.bash
cd ros_ws
colcon build
source install/setup.bash
```

### Download the LLM Model

Download the Qwen 2.5 7B Instruct GGUF:
```bash
pip install huggingface-hub
huggingface-cli download Qwen/Qwen2.5-7B-Instruct-GGUF \
  qwen2.5-7b-instruct-q5_k_m-00001-of-00002.gguf \
  qwen2.5-7b-instruct-q5_k_m-00002-of-00002.gguf \
  --local-dir ros_ws/src/voice_asr/voice_asr/
```

### Pre-download Whisper ASR Model

```bash
python3 -c "import whisper; whisper.load_model('base.en')"
```

---

## Launching the System

```bash
bash launch_full_system.sh
```

This starts a `byobu` session running the Gazebo simulation, the Crazyswarm2 driver, the LLM interface servers, and the Web UI.

Open **http://localhost:5000** in your browser to command the drone using voice or text.
