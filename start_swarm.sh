#!/bin/bash

# --- Display / Wayland Compatibility ---
# Gazebo (Gz) GUI can fail to render on Wayland sessions.
# Force Qt to use X11 (XCB) backend as a workaround.
export QT_QPA_PLATFORM=xcb
export GZ_GUI_RENDER_ENGINE=ogre2

# =============================================
# Cleanup function — kills ALL simulation and
# ROS processes. Used at startup (to clear
# zombies from a previous run) and on exit.
# =============================================
cleanup() {
    echo ""
    echo "Cleaning up all simulation processes..."
    # PX4 SITL instances
    pkill -9 -f "px4" 2>/dev/null
    # Gz Sim (Ignition/Harmonic) server & GUI
    pkill -9 -f "gz sim" 2>/dev/null
    pkill -9 -f "parameter_bridge" 2>/dev/null
    pkill -9 ruby 2>/dev/null
    # Gazebo Classic (just in case)
    pkill -9 gzserver 2>/dev/null
    pkill -9 gzclient 2>/dev/null
    # MAVSDK
    pkill -9 mavsdk_server 2>/dev/null
    # Our ROS 2 nodes
    pkill -9 -f "swarm_" 2>/dev/null
    # Wait for everything to actually die
    sleep 2
    echo "Cleanup complete."
}

# Trap EXIT, SIGINT (Ctrl+C), and SIGTERM so cleanup ALWAYS runs
trap cleanup EXIT INT TERM

echo "====================================="
echo "   Skyflock Swarm Startup Script     "
echo "====================================="

# 1. Clean up old processes to prevent conflicts
echo "[1/4] Cleaning up old simulation processes..."
cleanup

# 2. Launch PX4 Simulators in the background
echo "[2/4] Launching Gazebo and 3 PX4 Drones..."
cd ~/PX4-Autopilot

# Drone 1 (Starts Gazebo server + GUI)
make px4_sitl gz_x500 > /tmp/px4_drone1.log 2>&1 &
PX4_PID1=$!
sleep 25 # Give Gazebo time to fully start

# Drone 2
PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="5,0,0,0,0,0" build/px4_sitl_default/bin/px4 -i 1 build/px4_sitl_default/etc > /tmp/px4_drone2.log 2>&1 &
PX4_PID2=$!
sleep 10

# Drone 3
PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="10,0,0,0,0,0" build/px4_sitl_default/bin/px4 -i 2 build/px4_sitl_default/etc > /tmp/px4_drone3.log 2>&1 &
PX4_PID3=$!

# 3. Wait for EKF Sensors
echo "[3/4] Waiting 60 seconds for EKF sensors to stabilize..."
sleep 60

# 4. Launch ROS 2 Nodes
echo "[4/4] Launching ROS 2 Swarm Brain..."
cd ~/remaking_skyflock
source install/setup.bash

export NUM_DRONES=3

ros2 run my_digirc swarm_controller.py &
sleep 2
ros2 run my_digirc swarm_orchestrator.py &
sleep 2
ros2 run my_digirc swarm_telemetry_logger.py &
sleep 2

echo "====================================="
echo "   READY! Launching Keyboard...      "
echo "====================================="

# Run the keyboard controller in the foreground so you can interact with it.
# When you press Ctrl+C, the EXIT trap fires and cleanup() runs automatically.
ros2 run my_digirc swarm_keyboard_controller.py
