"""Known-height, camera-independent fingertip check. Preview unless --execute."""
import argparse
import json
from pathlib import Path
import signal
import time
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from scipy.spatial.transform import Rotation
from brsd_msgs.msg import DualArmStatus
from .config import load, motion_scales
from .task_node import PressTask, driver_config_match


def diagnostic_move(node, target, stage='HEIGHT_MOVE'):
    """Observe a non-contact command's settled error without declaring arrival."""
    from brsd_msgs.msg import DualArmMovePJ
    node.health(moving=True)
    if node.status.current_state!=DualArmStatus.IDLE:
        raise RuntimeError('Driver must be idle before diagnostic motion')
    pub = node.create_publisher(DualArmMovePJ, '/dual_arm/move_pj', 1)
    try:
        node.wait(lambda: pub.get_subscription_count()==1, 5, 'one motion subscriber')
        msg = DualArmMovePJ()
        msg.header.frame_id = node.cfg['base_frame']
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.arm_mask = 1 if node.arm=='left' else 2
        for key, value in zip(('x','y','z','rx','ry','rz'), target['xyz']+target['rpy']):
            setattr(msg.pose, node.arm+'_'+key, float(value))
        msg.speed, msg.a = motion_scales(node.cfg['motion'], stage)
        initial_joints = np.asarray(getattr(node.status.current_joint, node.arm+'_positions'), float)
        actual_joint_change = 0.
        seen_motion = False
        idle_since = None
        started = time.monotonic()
        node.event(stage, wrist_target=target, speed_scale=msg.speed, acceleration_scale=msg.a)
        pub.publish(msg)
        while time.monotonic()-started<90:
            rclpy.spin_once(node, timeout_sec=.04)
            node.health(moving=True)
            current_joints = np.asarray(getattr(node.status.current_joint, node.arm+'_positions'), float)
            actual_joint_change = max(actual_joint_change, float(np.max(np.abs(current_joints-initial_joints))))
            if node.waist.authority_weight < .98:
                raise RuntimeError('Upper-body authority weight fell below 0.98')
            seen_motion |= node.status.current_state==DualArmStatus.MOVING
            if seen_motion and node.status.current_state==DualArmStatus.IDLE:
                idle_since = idle_since or time.monotonic()
                if time.monotonic()-idle_since>=2:
                    if stage=='HEIGHT_MOVE' and actual_joint_change < .1:
                        raise RuntimeError('No substantial measured joint motion; do not count this as a height test')
                    node.event('HEIGHT_SETTLED', max_measured_joint_change_rad=actual_joint_change, wrist_target=target, measured_endpoint_reached=bool(node.close_to(target)))
                    return
            else:
                idle_since = None
        raise RuntimeError('Diagnostic motion did not finish in 90 seconds')
    finally:
        node.destroy_publisher(pub)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', choices=('shoulder', 'camera'), default='shoulder')
    parser.add_argument('--arm', choices=('left', 'right'), default='right')
    parser.add_argument('--hold', type=float, default=30.)
    parser.add_argument('--execute', action='store_true')
    args, ros_args = parser.parse_known_args()
    if not 1 <= args.hold <= 120:
        parser.error('--hold must be between 1 and 120 seconds')
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    stopping = [False]
    def interrupted(signum, frame):
        if not stopping[0]:
            stopping[0] = True
            raise KeyboardInterrupt()
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    node = None
    error = None
    try:
        cfg = load()
        node = PressTask(cfg, 'known_height', args.arm)
        reference_link = args.arm+'_shoulder_pitch_link' if args.reference=='shoulder' else cfg['camera']['mount_link']
        # At zero shoulder pitch its link origin is the fixed joint centre.
        positions = {args.arm+'_shoulder_pitch_joint': 0.}
        reference = node.robot.relative(cfg['base_frame'], reference_link, positions)[:3, 3]
        tip = node.robot.wrist_tip(cfg['tools'][args.arm])
        tcp = np.array([.38, -.25 if args.arm=='right' else .25, reference[2]])
        target = {'tcp_xyz': tcp.tolist(), 'xyz': (tcp-tip).tolist(), 'rpy': [0., 0., 0.]}
        node.event('HEIGHT_PREVIEW', reference_link=reference_link,
                   reference_xyz=reference.tolist(), wrist_target=target,
                   wrist_to_tip=tip.tolist(), hold_seconds=args.hold,
                   execute=args.execute, calibration_verified=False,
                   note='Torso-frame height; compare physically while torso is upright. No vision input.')
        expected = cfg['motion']['driver_config']
        matches = []
        for proc in Path('/proc').glob('[0-9]*/cmdline'):
            try:
                match = driver_config_match(proc.read_bytes().decode().split('\0'), expected)
            except (OSError, UnicodeError):
                continue
            if match is not None:
                matches.append(match)
        if matches != [True]:
            raise RuntimeError('Start exactly one rubber-hand business driver: scripts/ele.sh driver real')
        node.wait(lambda: node.status is not None and node.waist is not None, 8, 'driver feedback')
        node.health()
        if node.status.current_state not in (DualArmStatus.IDLE, DualArmStatus.DISABLED):
            raise RuntimeError('Driver must be stationary and idle or awaiting authority')
        velocities = list(node.status.current_joint.left_velocities)+list(node.status.current_joint.right_velocities)
        if not velocities or not np.all(np.isfinite(velocities)) or max(map(abs, velocities)) > cfg['motion']['max_joint_speed']:
            raise RuntimeError('Stationary measured arms are required')
        names = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')
        values = getattr(node.status.current_joint, args.arm+'_positions')
        if len(values)!=7 or not np.all(np.isfinite(values)):
            raise RuntimeError('Missing arm joint feedback')
        joints = {args.arm+'_'+name+'_joint': value for name, value in zip(names, values)}
        fk = node.robot.relative(cfg['base_frame'], cfg['tools'][args.arm]['wrist_link'], joints)
        measured = node.current_pose()
        if not np.all(np.isfinite(measured['xyz']+measured['rpy'])):
            raise RuntimeError('Invalid measured wrist pose')
        position_error = float(np.linalg.norm(fk[:3,3]-measured['xyz']))
        angle_error = (Rotation.from_matrix(fk[:3,:3]).inv()*Rotation.from_euler('XYZ', measured['rpy'])).magnitude()
        node.event('HEIGHT_MODEL_CHECK', wrist_target=target, fk_driver_position_error_m=position_error,
                   fk_driver_orientation_error_rad=float(angle_error), waist_radians=list(node.waist.current_joint.positions))
        if position_error>.01 or angle_error>.05:
            raise RuntimeError('Business model and driver forward kinematics disagree; inspect before motion')
        if not args.execute:
            return
        cfg['motion']['command_timeout_s'] = 90.
        initial = node.current_pose()
        initial_joint_target = {arm:list(getattr(node.status.current_joint, arm+'_positions')) for arm in ('left','right')}
        if not node.waist.has_upper_body_authority:
            node.acquired = True  # Also attempt release if authority acquisition is interrupted.
            node.trigger('/dual_arm/start')
            node.wait(lambda: node.waist.has_upper_body_authority, 5, 'authority feedback')
        node.wait(lambda: node.status.current_state==DualArmStatus.IDLE and node.waist.authority_weight>=.98, 5, 'idle and full authority after acquisition')
        node.executing = True
        diagnostic_move(node, target)
        node.event('HEIGHT_HOLD', reference_link=reference_link, wrist_target=target, hold_seconds=args.hold)
        settled = node.current_pose()
        until = time.monotonic()+args.hold
        last = 0.
        while time.monotonic()<until:
            rclpy.spin_once(node, timeout_sec=.04)
            node.health(moving=True)
            if np.linalg.norm(np.asarray(node.current_pose()['xyz'])-settled['xyz'])>.03:
                raise RuntimeError('Diagnostic holding pose drifted by more than 30 mm')
            if time.monotonic()-last>2:
                last = time.monotonic()
                node.event('HEIGHT_SAMPLE', wrist_target=target)
        node.move_joints(initial_joint_target, 'HEIGHT_RETURN')
        node.event('HEIGHT_COMPLETED', calibration_verified=False)
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc) or 'Interrupted'
        if node:
            node.event('ABORTED', reason=error)
            if node.executing:
                try:
                    node.cancel_motion()
                except Exception as cancel_error:
                    node.get_logger().error('Cancellation failed: '+str(cancel_error))
    finally:
        if node:
            if node.acquired:
                try:
                    node.trigger('/dual_arm/end')
                except Exception as release_error:
                    error = error or str(release_error)
                    node.get_logger().error('Authority release failed: '+str(release_error))
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if error:
        raise SystemExit(error)


if __name__ == '__main__':
    main()
