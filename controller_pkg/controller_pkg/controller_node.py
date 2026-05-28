from crazyflie_py import Crazyswarm
import rclpy


ZERO_VECTOR = [0.0, 0.0, 0.0]


def _declare_and_read_parameters(node):
    node.declare_parameter('cf_name', 'cf_0')
    node.declare_parameter('x', 0.0)
    node.declare_parameter('y', 0.0)
    node.declare_parameter('z', 1.0)
    node.declare_parameter('yaw', 0.0)
    node.declare_parameter('takeoff_height', 0.5)
    node.declare_parameter('takeoff_duration', 2.0)
    node.declare_parameter('stream_rate', 20.0)
    node.declare_parameter('hold_duration', -1.0)
    node.declare_parameter('land_height', 0.04)
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

    took_off = False
    try:
        if params['force_oot_controller']:
            cf.setParam('stabilizer.controller', params['oot_controller_id'])
            time_helper.sleep(0.2)
            allcfs.get_logger().info(
                'Requested stabilizer.controller=%d for %s.'
                % (params['oot_controller_id'], params['cf_name'])
            )

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
        time_helper.sleep(params['takeoff_duration'] + 1.0)
        took_off = True

        start_time = time_helper.time()
        while rclpy.ok():
            elapsed = time_helper.time() - start_time
            if params['hold_duration'] >= 0.0 and elapsed >= params['hold_duration']:
                break

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
            cf.notifySetpointsStop(remainValidMillisecs=100)
            time_helper.sleep(0.2)
            cf.land(params['land_height'], params['land_duration'])
            time_helper.sleep(params['land_duration'] + 0.5)

        if rclpy.ok():
            rclpy.shutdown()
