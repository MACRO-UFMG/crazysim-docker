# crazysim-docker

Terminal 1:
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_text.sh -f agents.txt -m crazyflie
ros2 launch crazyflie launch.py backend:=cflib

Terminal 2:

ros2 service call /all/takeoff crazyflie_interfaces/srv/Takeoff "{height: 0.5, duration: {sec: 2, nanosec: 0}}"

ros2 service call /all/land crazyflie_interfaces/srv/Land "{height: 0.0, duration: {sec: 2, nanosec: 0}}"


