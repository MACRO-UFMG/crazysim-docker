# crazysim-docker

ros2 launch crazyflie launch.py backend:=cflib

make sure `\CrazySim\crazyswarm2_ws\src\crazyswarm2\crazyflie\config\crazyflies.yaml` is the same as the one in the repo

ros2 service call /all/takeoff crazyflie_interfaces/srv/Takeoff "{height: 0.5, duration: {sec: 2, nanosec: 0}}"

ros2 service call /all/land crazyflie_interfaces/srv/Land "{height: 0.0, duration: {sec: 2, nanosec: 0}}"

ros2 launch visibility_guard tunnel_crazysim.launch.py 
ros2 launch visibility_guard my_lab_tunnel_crazysim.launch.py 

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/ref_vel