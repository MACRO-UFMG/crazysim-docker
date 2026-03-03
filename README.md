# crazysim-docker

Terminal 1:

ros2 launch crazyflie launch.py backend:=cflib

Terminal 2:

ros2 service call /all/takeoff crazyflie_interfaces/srv/Takeoff "{height: 0.5, duration: {sec: 2, nanosec: 0}}"

Terminal 3:
cd /CrazySim/crazyswarm2_ws/src/visibility-guard
ros2 bag record -a

Terminal 4:
ros2 launch visibility_guard cluttered_crazysim.launch.py 

Terminal 5:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/ref_vel

Para comandar o líder, ative o CAPS LOCK e pressione I










ros2 service call /all/land crazyflie_interfaces/srv/Land "{height: 0.0, duration: {sec: 2, nanosec: 0}}"

ros2 launch visibility_guard my_lab_tunnel_crazysim.launch.py 

