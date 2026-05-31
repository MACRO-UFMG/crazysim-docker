from crazyflie_py import Crazyswarm
from geometry_msgs.msg import PoseStamped
import rclpy
from rcl_interfaces.msg import SetParametersResult
import numpy as np
from abc import ABC, abstractmethod


ZERO_VECTOR = [0.0, 0.0, 0.0]


def golden_search(f, xl: float, xu: float, it_max: int = 1e6, et : float = 1e-6):
    it = 0
    e = 1
    def update_interior(xl,xu):
        d = ((np.sqrt(5) - 1)/2)*(xu - xl)
        x1 = xl+d
        x2 = xu-d
        return (x1,x2)
    while e >= et and it <= it_max:
        x1,x2 = update_interior(xl, xu)
        # if mode == 'max':
        #     xl,xu,xopt = find_max(xl,xu,x1,x2,label)
        # else:
        fx1 = f(x1)
        fx2 = f(x2)
        if fx2 > fx1 and x2 < x1:
            xl = x2
            xu = xu
            x1,x2 = update_interior(xl, xu)
            xopt = x1
        else:
            xl = xl
            xu = x1
            x1,x2 = update_interior(xl, xu)
            xopt = x2
        it += 1
        r = (np.sqrt(5)-1)/2
        # e = ((1-r)*(abs((xu-xl)/xopt)))*100 
        e = abs(xu-xl)
    return (f(xopt), xopt)


def derivative(f, x: np.array, delta: float = 0.001):
    f0 = f(x)
    if np.isscalar(x):
        df_dx = (1/delta)*( f(x + delta/2) - f(x - delta/2) )
    else:
        n = x.size
        df_dx = np.zeros((f0.shape[0], n))
        for i in range(n):
            dx = np.zeros_like(x)
            dx[i] = delta/2
            df_dx[:,i] = (1/delta)*( f(x + dx) - f(x - dx) )
    if f0.shape[0] == 1:
        df_dx = df_dx.flatten()
    return df_dx


class Commander(ABC):
    @abstractmethod
    def compute_command(self, pose, t):
        pass


class ZeroCommander(Commander):
    def compute_command(self, pose, t):
        return ZERO_VECTOR, ZERO_VECTOR, ZERO_VECTOR, 0.0, ZERO_VECTOR


class PathCommander(Commander):
    def __init__(self, path):
        self._path = path
        self._s_star = None
        self._delta_s = np.pi/4

    def compute_command(self, pose, t):
        p = np.array([pose[0], pose[1], pose[2]]) if pose is not None else np.array([[0.0, 0.0, 0.0]])
        if self._s_star is not None:
            s_search1 = self._s_star - self._delta_s
            s_search2 = self._s_star + self._delta_s
        else:  
            s_search1 = 0
            s_search2 = 2*np.pi
        _,self._s_star = golden_search(lambda s: (np.linalg.norm(p - self._path(s,t)))**2, s_search1, s_search2)
        c_star = self._path(self._s_star,t)
        # D = p - c_star
        def compute_T(s,t):
            return derivative(lambda ss: self._path(ss,t), s)
        T = compute_T(self._s_star,t)
        pos = [c_star[0], c_star[1], c_star[2]]
        vel = [T[0], T[1], T[2]]
        return pos, vel, ZERO_VECTOR, 0.0, ZERO_VECTOR


class ControllerNode():
    def __init__(self, commander = ZeroCommander()):
        swarm = Crazyswarm()
        self._node = swarm.allcfs
        self._time_helper = swarm.timeHelper
        self._params = self._declare_and_read_parameters()
        self._node.add_on_set_parameters_callback(self._on_parameter_update)
        self._cf = self._node.crazyfliesByName.get(self._params['cf_name'])
        if self._cf is None:
            available = ', '.join(sorted(self._node.crazyfliesByName)) or 'none'
            self._node.get_logger().error(
                f"Crazyflie '{self._params['cf_name']}' was not found. Available robots: {available}"
            )
            if rclpy.ok():
                rclpy.shutdown()
            raise SystemExit(1)
        self._pose = None
        self._commander = commander

    def run(self):
        took_off = False
        try:
            pose_topic = f"/{self._params['cf_name']}/pose"
            self._node.create_subscription(PoseStamped, pose_topic, self._pose_callback, 10)
            self._node.get_logger().info(f"Subscribed to pose topic: {pose_topic}")
            self._node.get_logger().info(
                'Taking off %s to %.2f m and streaming full-state command '
                '(x=%.2f, y=%.2f, z=%.2f, yaw=%.2f rad) at %.1f Hz.'
                % (
                    self._params['cf_name'],
                    self._params['takeoff_height'],
                    self._params['target'][0],
                    self._params['target'][1],
                    self._params['target'][2],
                    self._params['yaw'],
                    self._params['stream_rate'],
                )
            )
            self._cf.takeoff(self._params['takeoff_height'], self._params['takeoff_duration'])
            self._time_helper.sleep(self._params['takeoff_duration'])
            took_off = True
            self._node.get_logger().info('Took off complete. Holding position.')
            if self._params['hold_duration'] >= 0.0:
                self._time_helper.sleep(self._params['hold_duration'])
            self._node.get_logger().info('Hover complete. Starting full-state command streaming.')
            start_time = self._time_helper.time()
            while rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.0)
                elapsed = self._time_helper.time() - start_time
                pos, vel, acc, yaw, omega = self._commander.compute_command(self._pose, elapsed)
                self._node.get_logger().info(f'Sending full-state command to {self._params["cf_name"]}')
                self._cf.cmdFullState(
                    pos,
                    vel,
                    acc,
                    yaw,
                    omega,
                )
                self._time_helper.sleepForRate(self._params['stream_rate'])
        except KeyboardInterrupt:
            self._node.get_logger().info('Interrupted. Releasing low-level setpoints and landing.')
        finally:
            if took_off and rclpy.ok():
                self._cf.notifySetpointsStop(remainValidMillisecs=500)
                self._time_helper.sleep(0.2)
                self._cf.land(self._params['land_height'], self._params['land_duration'])
                self._time_helper.sleep(self._params['land_duration'])
            if rclpy.ok():
                rclpy.shutdown()

    def _declare_and_read_parameters(self):
        self._node.declare_parameter('cf_name', 'cf_0')
        self._node.declare_parameter('x', -0.5)
        self._node.declare_parameter('y', 1.0)
        self._node.declare_parameter('z', 2.0)
        self._node.declare_parameter('yaw', 0.0)
        self._node.declare_parameter('takeoff_height', 0.5)
        self._node.declare_parameter('takeoff_duration', 1.0)
        self._node.declare_parameter('stream_rate', 10.0)
        self._node.declare_parameter('hold_duration', 1.0)
        self._node.declare_parameter('land_height', 0.1)
        self._node.declare_parameter('land_duration', 2.0)

        return {
            'cf_name': self._node.get_parameter('cf_name').value,
            'target': [
                float(self._node.get_parameter('x').value),
                float(self._node.get_parameter('y').value),
                float(self._node.get_parameter('z').value),
            ],
            'yaw': float(self._node.get_parameter('yaw').value),
            'takeoff_height': float(self._node.get_parameter('takeoff_height').value),
            'takeoff_duration': float(self._node.get_parameter('takeoff_duration').value),
            'stream_rate': float(self._node.get_parameter('stream_rate').value),
            'hold_duration': float(self._node.get_parameter('hold_duration').value),
            'land_height': float(self._node.get_parameter('land_height').value),
            'land_duration': float(self._node.get_parameter('land_duration').value),
        }

    def _on_parameter_update(self, updated_parameters):
        for param in updated_parameters:
            name = param.name
            if name == 'x':
                self._params['target'][0] = float(param.value)
            elif name == 'y':
                self._params['target'][1] = float(param.value)
            elif name == 'z':
                self._params['target'][2] = float(param.value)
            elif name == 'yaw':
                self._params['yaw'] = float(param.value)
            elif name == 'stream_rate':
                stream_rate = float(param.value)
                if stream_rate <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='stream_rate must be > 0.0',
                    )
                self._params['stream_rate'] = stream_rate
            elif name == 'hold_duration':
                self._params['hold_duration'] = float(param.value)
            elif name in {
                'takeoff_height',
                'takeoff_duration',
                'land_height',
                'land_duration',
            }:
                self._params[name] = float(param.value)
        allcfs.get_logger().info(
            'Updated params: target=(%.2f, %.2f, %.2f), yaw=%.2f, stream_rate=%.1f Hz'
            % (
                self._params['target'][0],
                self._params['target'][1],
                self._params['target'][2],
                self._params['yaw'],
                self._params['stream_rate'],
            )
        )
        return SetParametersResult(successful=True)

    def _pose_callback(self, msg: PoseStamped):
        self._pose = [
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
        ]


circle_path = lambda s,t: np.array([
    0.5*np.cos(s),
    0.5*np.sin(s),
    1.0
])


def main():
    cn = ControllerNode(commander=PathCommander(circle_path))
    cn.run()
