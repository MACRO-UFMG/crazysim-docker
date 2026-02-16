# crazysim-docker

ros2 launch crazyflie launch.py backend:=cflib

make sure `\CrazySim\crazyswarm2_ws\src\crazyswarm2\crazyflie\config\crazyflies.yaml` is the same as the one in the repo

ros2 service call /all/takeoff crazyflie_interfaces/srv/Takeoff "{height: 0.5, duration: {sec: 2, nanosec: 0}}"