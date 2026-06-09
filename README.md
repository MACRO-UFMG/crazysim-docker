# crazysim-docker

Terminal 0:

```bash
cd /CrazySim/crazyflie-firmware
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_text.sh -f agents.txt -m crazyflie
```

Terminal 1:

```bash
ros2 launch crazyflie launch.py backend:=cflib gui:=false
```

Terminal 2:

```bash
ros2 service call /all/takeoff crazyflie_interfaces/srv/Takeoff "{height: 0.5, duration: {sec: 2, nanosec: 0}}"
```

```bash
ros2 service call /all/land crazyflie_interfaces/srv/Land "{height: 0.0, duration: {sec: 2, nanosec: 0}}"
```

Terminal 3:

```bash
cd /CrazySim/crazyswarm2_ws/src/visibility-guard
ros2 bag record -a
```

Terminal 4:

```bash
ros2 launch visibility_guard cluttered_crazysim.launch.py
```

Terminal 5:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/ref_vel
```

Para comandar o líder, ative o CAPS LOCK e pressione I
