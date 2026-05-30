from crazyflie_py import Crazyswarm
from geometry_msgs.msg import PoseStamped
import rclpy
from rcl_interfaces.msg import SetParametersResult
import numpy as np


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


target_curve = lambda s,t: np.array([
    0.5*np.cos(s),
    0.5*np.sin(s),
    1.0
])


def control_input(p, t, s_star_prev=None):
    delta_s = np.pi/4
    if s_star_prev is not None:
        s_search1 = s_star_prev - delta_s
        s_search2 = s_star_prev + delta_s
    else:  
        s_search1 = 0
        s_search2 = 2*np.pi
    _,s_star = golden_search(lambda s: (np.linalg.norm(p - target_curve(s,t)))**2, s_search1, s_search2)
    c_star = target_curve(s_star,t)
    D = p - c_star
    def compute_T(s,t):
        return derivative(lambda ss: target_curve(ss,t), s)
    T = compute_T(s_star,t)
    return s_star, c_star, D, T



def _declare_and_read_parameters(node):
    node.declare_parameter('cf_name', 'cf_0')
    node.declare_parameter('x', -0.5)
    node.declare_parameter('y', 1.0)
    node.declare_parameter('z', 2.0)
    node.declare_parameter('yaw', 0.0)
    node.declare_parameter('takeoff_height', 0.5)
    node.declare_parameter('takeoff_duration', 2.0)
    node.declare_parameter('stream_rate', 10.0)
    node.declare_parameter('hold_duration', 1.0)
    node.declare_parameter('land_height', 0.1)
    node.declare_parameter('land_duration', 2.0)
    node.declare_parameter('force_oot_controller', True)
    node.declare_parameter('oot_controller_id', 5)

    return {
        'cf_name': node.get_parameter('cf_name').value,
        'target': [
            float(node.get_parameter('x').value),
            float(node.get_parameter('y').value),
            float(node.get_parameter('z').value),
        ],
        'yaw': float(node.get_parameter('yaw').value),
        'takeoff_height': float(node.get_parameter('takeoff_height').value),
        'takeoff_duration': float(node.get_parameter('takeoff_duration').value),
        'stream_rate': float(node.get_parameter('stream_rate').value),
        'hold_duration': float(node.get_parameter('hold_duration').value),
        'land_height': float(node.get_parameter('land_height').value),
        'land_duration': float(node.get_parameter('land_duration').value),
        'force_oot_controller': bool(node.get_parameter('force_oot_controller').value),
        'oot_controller_id': int(node.get_parameter('oot_controller_id').value),
    }


def main():
    swarm = Crazyswarm()
    allcfs = swarm.allcfs
    time_helper = swarm.timeHelper
    params = _declare_and_read_parameters(allcfs)

    cf = allcfs.crazyfliesByName.get(params['cf_name'])
    if cf is None:
        available = ', '.join(sorted(allcfs.crazyfliesByName)) or 'none'
        allcfs.get_logger().error(
            f"Crazyflie '{params['cf_name']}' was not found. Available robots: {available}"
        )
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(1)

    latest_pose = {'value': None}

    def _pose_callback(msg: PoseStamped):
        latest_pose['value'] = [
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        ]

    pose_topic = f"/{params['cf_name']}/pose"
    allcfs.create_subscription(PoseStamped, pose_topic, _pose_callback, 10)
    allcfs.get_logger().info(f"Subscribed to pose topic: {pose_topic}")

    def _on_parameter_update(updated_parameters):
        for param in updated_parameters:
            name = param.name

            if name == 'x':
                params['target'][0] = float(param.value)
            elif name == 'y':
                params['target'][1] = float(param.value)
            elif name == 'z':
                params['target'][2] = float(param.value)
            elif name == 'yaw':
                params['yaw'] = float(param.value)
            elif name == 'stream_rate':
                stream_rate = float(param.value)
                if stream_rate <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='stream_rate must be > 0.0',
                    )
                params['stream_rate'] = stream_rate
            elif name == 'hold_duration':
                params['hold_duration'] = float(param.value)
            elif name == 'force_oot_controller':
                params['force_oot_controller'] = bool(param.value)
                if params['force_oot_controller']:
                    cf.setParam('stabilizer.controller', int(params['oot_controller_id']))
            elif name == 'oot_controller_id':
                params['oot_controller_id'] = int(param.value)
                if params['force_oot_controller']:
                    cf.setParam('stabilizer.controller', params['oot_controller_id'])
            elif name in {
                'takeoff_height',
                'takeoff_duration',
                'land_height',
                'land_duration',
            }:
                params[name] = float(param.value)

        allcfs.get_logger().info(
            'Updated params: target=(%.2f, %.2f, %.2f), yaw=%.2f, stream_rate=%.1f Hz'
            % (
                params['target'][0],
                params['target'][1],
                params['target'][2],
                params['yaw'],
                params['stream_rate'],
            )
        )

        return SetParametersResult(successful=True)

    allcfs.add_on_set_parameters_callback(_on_parameter_update)

    took_off = False
    try:
        # if params['force_oot_controller']:
        #     param_client = AsyncParametersClient('/crazyflie_server')

        #     param_client.wait_for_services()

        #     param_client.set_parameters([
        #         Parameter(
        #             'all.params.stabilizer.controller',
        #             Parameter.Type.INTEGER,
        #             params['oot_controller_id']
        #         )
        #     ])

        allcfs.get_logger().info(
            'Taking off %s to %.2f m and streaming full-state command '
            '(x=%.2f, y=%.2f, z=%.2f, yaw=%.2f rad) at %.1f Hz.'
            % (
                params['cf_name'],
                params['takeoff_height'],
                params['target'][0],
                params['target'][1],
                params['target'][2],
                params['yaw'],
                params['stream_rate'],
            )
        )

        cf.takeoff(params['takeoff_height'], params['takeoff_duration'])
        time_helper.sleep(params['takeoff_duration'])
        took_off = True
        allcfs.get_logger().info('Took off complete. Holding position.')
        if params['hold_duration'] >= 0.0:
            time_helper.sleep(params['hold_duration'])
        allcfs.get_logger().info('Hover complete. Starting full-state command streaming.')
        start_time = time_helper.time()
        s_star = None
        while rclpy.ok():
            rclpy.spin_once(allcfs, timeout_sec=0.0)
            elapsed = time_helper.time() - start_time
            
            allcfs.get_logger().info('Sending full-state command to %s: pos=(%.2f, %.2f, %.2f), yaw=%.2f rad' % (
                params['cf_name'],
                params['target'][0],
                params['target'][1],
                params['target'][2],
                params['yaw']
            ))

            p = np.array([latest_pose['value']]) if latest_pose['value'] is not None else np.array([[0.0, 0.0, 0.0]])
            s_star, c_star, D, T = control_input(p, elapsed, s_star_prev=s_star)

            cmd_pos = [c_star[0], c_star[1], c_star[2]]
            cmd_vel = [T[0], T[1], T[2]]

            cf.cmdFullState(
                cmd_pos,
                cmd_vel,
                ZERO_VECTOR,
                params['yaw'],
                ZERO_VECTOR,
            )
            time_helper.sleepForRate(params['stream_rate'])
    except KeyboardInterrupt:
        allcfs.get_logger().info('Interrupted. Releasing low-level setpoints and landing.')
    finally:
        if took_off and rclpy.ok():
            cf.notifySetpointsStop(remainValidMillisecs=500)
            time_helper.sleep(0.2)
            cf.land(params['land_height'], params['land_duration'])
            time_helper.sleep(params['land_duration'])
        if rclpy.ok():
            rclpy.shutdown()
