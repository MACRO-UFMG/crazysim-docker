#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${COMPOSE_FILE:-$ROOT_DIR/docker-compose.yaml}"
SERVICE="${CRAZYSIM_SERVICE:-crazysim}"
BUILD_JOBS="${BUILD_JOBS:-12}"
USE_NVIDIA="${USE_NVIDIA:-true}"

if [[ "$USE_NVIDIA" == "true" ]]; then
    DOCKER_COMPOSE=(docker compose -f "$COMPOSE_FILE" -f "$ROOT_DIR/compose.d/nvidia-gpu.yaml" -f "$ROOT_DIR/compose.d/hack.yaml")
else
    DOCKER_COMPOSE=(docker compose -f "$COMPOSE_FILE" -f "$ROOT_DIR/compose.d/hack.yaml")
fi

print_help() {
  cat <<'EOF'
CrazySim helper script

Usage:
  ./crazysim.sh <command> [args]

Container lifecycle:
  up                      Start or recreate container in detached mode
  stop                    Stop container
  down                    Stop and remove stack
  restart                 Restart container
  status                  Show compose status
  logs                    Tail container logs
  shell                   Open bash shell inside container

Simulation and ROS:
  sim-start               Start SITL/Gazebo multi-agent simulator in container
  ros-launch [backend]    Launch Crazyswarm2 server (default backend: cflib)
  node-launch [args]      Run fixed_setpoint_node
                          args: <cf_name> <x> <y> <z> <yaw> <hold_duration>
                          defaults: cf_0 0.0 0.0 0.5 0.0 -1.0

Build commands:
  build-oot               Build app_my_controller for SITL
  build-sitl              Build crazyflie-firmware sitl_make target
  build-fw                Run build-oot + build-sitl
  build-ws [pkg|all]      Build ROS2 workspace package (default: fixed_setpoint_controller)

Environment overrides:
  COMPOSE_FILE            Compose file path (default: ./docker-compose.yaml)
  CRAZYSIM_SERVICE        Compose service name (default: crazysim)
  BUILD_JOBS              Parallel jobs for make (default: 12)

Examples:
  ./crazysim.sh up
  ./crazysim.sh sim-start
  ./crazysim.sh ros-launch cflib
  ./crazysim.sh node-launch cf_0 0.0 0.0 0.8 0.0 10.0
  ./crazysim.sh build-fw
  ./crazysim.sh build-ws all
EOF
}

dc() {
  (cd "$ROOT_DIR" && "${DOCKER_COMPOSE[@]}" "$@")
}

dc_up() {
    dc up -d --force-recreate "$SERVICE"
}

dc_down() {
    dc down
}

ensure_container_running() {
  if ! dc ps --status running --services | grep -qx "$SERVICE"; then
    echo "Container service '$SERVICE' is not running. Starting it first..." >&2
    dc_up
  fi
}

exec_in_container() {
  local cmd="$1"
  ensure_container_running
  dc exec "$SERVICE" bash -i -c "$cmd"
}

sim_start() {
    xhost +local:root
    exec_in_container "cd /CrazySim/crazyflie-firmware && bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_text.sh -f agents.txt -m crazyflie"
}

sim_start_bg() {
    xhost +local:root
    ensure_container_running
    dc exec "$SERVICE" bash -i -c "cd /CrazySim/crazyflie-firmware && bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_text.sh -f agents.txt -m crazyflie" > /tmp/sim_start.log 2>&1 &
    echo "Simulator starting in background (PID: $!). Log: tail -f /tmp/sim_start.log"
}

ros_launch() {
    backend="${1:-cflib}"
    exec_in_container "cd /CrazySim/crazyswarm2_ws && source install/setup.bash && ros2 launch crazyflie launch.py backend:=$backend"
}

build_sitl() {
    exec_in_container "cd /CrazySim/crazyflie-firmware/sitl_make/build && cmake .. && make -j $BUILD_JOBS all"
}

build_ws() {
    pkg="${1:-controller_pkg}"
    if [[ "$pkg" == "all" ]]; then
      exec_in_container "cd /CrazySim/crazyswarm2_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash"
    else
      exec_in_container "cd /CrazySim/crazyswarm2_ws && colcon build --symlink-install --packages-select $pkg && source install/setup.bash"
    fi
}

cmd="${1:-help}"
shift || true

case "$cmd" in
  help|-h|--help)
    print_help
    ;;
  up)
    dc_up
    ;;
  stop)
    dc stop "$SERVICE"
    ;;
  down)
    dc_down
    ;;
  restart)
    dc restart "$SERVICE"
    ;;
  status)
    dc ps
    ;;
  logs)
    dc logs -f "$SERVICE"
    ;;
  shell)
    ensure_container_running
    dc exec "$SERVICE" bash
    ;;
  sim-start)
    sim_start
    ;;
  ros-launch)
    ros_launch "$@"
    ;;
  take-off)
    exec_in_container "ros2 service call /all/takeoff crazyflie_interfaces/srv/Takeoff \"{height: 0.5, duration: {sec: 2, nanosec: 0}}\""
    ;;
  land)
    exec_in_container "ros2 service call /all/land crazyflie_interfaces/srv/Land \"{height: 0.0, duration: {sec: 2, nanosec: 0}}\""
    ;;
  node-launch)
    cf_name="${1:-cf_0}"
    x="${2:-0.0}"
    y="${3:-0.0}"
    z="${4:-0.5}"
    yaw="${5:-0.0}"
    hold_duration="${6:--1.0}"
    exec_in_container "source /CrazySim/crazyswarm2_ws/install/setup.bash && ros2 run controller_pkg fixed_setpoint_node --ros-args -p cf_name:=$cf_name -p x:=$x -p y:=$y -p z:=$z -p yaw:=$yaw -p hold_duration:=$hold_duration"
    ;;
  build-oot)
    exec_in_container "cd /CrazySim/app_my_controller && make PLATFORM=sitl EXTRA_CFLAGS=\"-Wno-error\" -j $BUILD_JOBS"
    ;;
  build-sitl)
    build_sitl
    ;;
  build-fw)
    exec_in_container "cd /CrazySim/app_my_controller && make PLATFORM=sitl EXTRA_CFLAGS=\"-Wno-error\" -j $BUILD_JOBS"
    build_sitl
    ;;
  build-ws)
    build_ws "$@"
    ;;
  full)
    dc_down
    dc_up
    build_ws
    build_sitl
    sim_start_bg
    echo "Waiting 15s for Gazebo to initialize..."
    sleep 15
    ros_launch cflib
    ;;
  *)
    echo "Unknown command: $cmd" >&2
    print_help
    exit 2
    ;;
esac
