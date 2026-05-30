from crazyflie_py import Crazyswarm
from geometry_msgs.msg import PoseStamped
import rclpy
from rcl_interfaces.msg import SetParametersResult


ZERO_VECTOR = [0.0, 0.0, 0.0]


def _declare_and_read_parameters(node):
    node.declare_parameter('cf_name', 'cf_0')
    node.declare_parameter('x', 0.0)
    node.declare_parameter('y', 0.0)
    node.declare_parameter('z', 1.0)
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

            cf.cmdFullState(
                params['target'],
                ZERO_VECTOR,
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
