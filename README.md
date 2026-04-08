# 🛸 CrazySim Docker Workspace

This repository provides a containerized environment for **CrazySim**, integrating ROS2 Humble, Gazebo Garden, and Crazyswarm2 for multi-quadrotor simulation.

---

## 🚀 Getting Started

### 1. Launch the Simulation (Terminal 1)
Initialize the Gazebo physics engine and the SITL (Software-In-The-Loop) instances for the drones defined in your `agents.txt`.

```bash
# 1a. Start Gazebo and spawn agents
cd /CrazySim/crazyflie-firmware
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_text.sh -f agents.txt -m crazyflie
```

Once Gazebo is running, launch the ROS2 server and telemetry GUI in the same container:

```bash
# 1b. Launch Crazyswarm2 server
ros2 launch crazyflie launch.py backend:=cflib
```
> **Note:** The `backend:=cflib` interfaces with the SITL instances. Ensure your `crazyflies.yaml` is correctly mapped to the URIs used in Gazebo.

---

### 2. Fleet Control Commands (Terminal 2)
Use these service calls to control the entire fleet (`/all`) or specific drones.

#### 🛫 Takeoff
Command all drones to rise to a height of 0.5 meters over 2 seconds:
```bash
ros2 service call /all/takeoff crazyflie_interfaces/srv/Takeoff "{height: 0.5, duration: {sec: 2, nanosec: 0}}"
```

#### 🛬 Land
Command all drones to descend to the ground:
```bash
ros2 service call /all/land crazyflie_interfaces/srv/Land "{height: 0.0, duration: {sec: 2, nanosec: 0}}"
```

#### 🛑 Emergency Stop
In case of unstable oscillations or collisions:
```bash
ros2 service call /all/emergency_stop std_srvs/srv/Empty "{}"
```

---

## 💻 Custom Firmware Development

### Building for SITL (Software-In-The-Loop)
If you are developing the custom Out-Of-Tree (OOT) controller or modifying the core Crazyflie firmware, you must recompile the native Linux executable used by the simulator. 

Run the following command to build the firmware for the SITL environment. The `EXTRA_CFLAGS="-Wno-error"` flag is necessary to bypass strict 64-bit compiler warnings when building inside the Docker container:

```bash
cd /CrazySim/app_my_controller
make PLATFORM=sitl EXTRA_CFLAGS="-Wno-error" -j 12

```

---

## 🛠 Troubleshooting & Maintenance

### 🐍 Python Environment Fix
If you encounter `AttributeError: _ARRAY_API not found` or `ImportError` related to **NiceGUI/NumPy**, force the environment back to the compatible ABI:
```bash
# Force-align NumPy with ROS2 Humble's ABI
pip install "numpy<2.0.0" --force-reinstall
```

### 🔨 Rebuilding the Workspace
If you modify C++ source code in `crazyswarm2_ws` or add new ROS2 packages:
```bash
cd /CrazySim/crazyswarm2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 📂 Project Structure
* **`agents.txt`**: Defines the number and initial coordinates of drones for the Gazebo world.
* **`crazyflies.yaml`**: ROS2 configuration for drone IDs, URIs, and logging channels.
* **`gui.py`**: The NiceGUI-based dashboard for real-time monitoring and high-level control.

---

## 🛰 Telemetry Monitoring
To monitor individual drone transforms or topics during flight:
```bash
# View the TF tree
ros2 run tf2_tools view_frames

# Listen to a specific drone's pose
ros2 topic echo /cf_0/pose
```