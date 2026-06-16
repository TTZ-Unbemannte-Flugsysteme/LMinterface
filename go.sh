#!/bin/bash
# Quick Build & Launch - One command to rule them all!
# Usage: ./go.sh        - Build and launch
#        ./go.sh quick  - Skip build, just launch
#        ./go.sh build  - Only build

set -e  # Exit on any error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="${SCRIPT_DIR}/ros_ws"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

build_workspace() {
    echo -e "${BLUE}🔨 Building ROS2 workspace...${NC}"
    cd "$ROS_WS"
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    echo -e "${GREEN}✓ Build complete!${NC}"
}

launch_system() {
    echo -e "${BLUE}🚀 Launching full system...${NC}"
    cd "$SCRIPT_DIR"
    ./launch_full_system.sh
}

case "${1:-full}" in
    quick|q)
        # Skip build, just launch
        launch_system
        ;;
    build|b)
        # Only build
        build_workspace
        echo -e "${GREEN}✓ Ready! Run './go.sh quick' to launch${NC}"
        ;;
    full|f|*)
        # Build and launch (default)
        build_workspace
        launch_system
        ;;
esac
