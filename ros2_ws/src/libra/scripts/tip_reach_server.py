#!/usr/bin/env python3

# Brief:
#   Serves the tip reach of a requested pose: the horizontal distance from the
#   pitch joint to the tip of the arm, using forward kinematics on the URDF that
#   robot_state_publisher latches on /robot_description.
#
# Usage:
#   $ ros2 run libra tip_reach_server.py
#   $ ros2 service call /libra/get_tip_reach libra_interfaces/srv/GetTipReach \
#         "{poses: [{name: [Roll, Pitch, J1, J2, J3], position: [0, 0, 0, 0, 0]}]}"
#
# Assumptions:
#   - The URDF's root frame has Z up, so "horizontal" is its XY plane.
#   - Each request specifies every movable joint between the root and the tip.

# TODO: check Claude-generated code

import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation
from std_msgs.msg import String

from libra_interfaces.srv import GetTipReach

TIP_LINK = 'arm_out_link'
BASE_JOINT = 'Pitch'


def origin_transform(origin):
    """
    Returns the 4x4 transform of a URDF <origin> element (identity if absent).
    """
    transform = np.eye(4)
    if origin is None:
        return transform

    rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
    transform[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()  # URDF: fixed axes
    transform[:3, 3] = [float(v) for v in origin.get('xyz', '0 0 0').split()]

    return transform


class TipReachServer(Node):
    """
    Computes the tip reach of requested poses from the robot description.
    """
    def __init__(self):
        super().__init__('tip_reach_server')

        self.joint_chain_ = None  # from root to TIP_LINK
        self.base_idx_ = None  # index of BASE_JOINT in the chain

        # Robot description is published once, so keep the latched copy
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.description_sub_ = self.create_subscription(
            String, '/robot_description', self.load_chain, latched)
        self.service_ = self.create_service(
            GetTipReach, '/libra/get_tip_reach', self.serve_reach)

        self.get_logger().info('Tip reach server started; waiting for /robot_description')

    def load_chain(self, msg):
        """
        Builds the joint chain from the URDF root to TIP_LINK.
        """
        # Map each link to its parent joint
        joints = {j.find('child').get('link'): j for j in ET.fromstring(msg.data).iter('joint')}
        if TIP_LINK not in joints:
            self.get_logger().error(f'Robot description has no link named {TIP_LINK}')
            return

        # Walk backwards from tip to root, collecting joints
        chain = []
        link = TIP_LINK
        while link in joints:
            joint = joints[link]
            axis = joint.find('axis')
            chain.append({
                'name': joint.get('name'),
                'movable': joint.get('type') != 'fixed',
                'origin': origin_transform(joint.find('origin')),
                'axis': np.array(axis.get('xyz').split(), float) if axis is not None else np.array([1.0, 0, 0]),
            })
            link = joint.find('parent').get('link')

        chain.reverse()  # get root to tip

        # Find base joint
        names = [joint['name'] for joint in chain]
        if BASE_JOINT not in names:
            self.get_logger().error(f'No {BASE_JOINT} joint between the root and {TIP_LINK}')
            return

        # Save chain
        self.joint_chain_ = chain
        self.base_idx_ = names.index(BASE_JOINT)
        movable = [joint['name'] for joint in chain if joint['movable']]
        self.get_logger().info(f'Loaded chain to {TIP_LINK} (joints: {", ".join(movable)})')

    def get_reach(self, angles):
        """
        Returns the horizontal distance from BASE_JOINT to the tip for joint
        angles given as `{name: rad}`.
        """
        # Apply each joint
        transform = np.eye(4)
        base_pos = None
        for index, joint in enumerate(self.joint_chain_):
            transform = transform @ joint['origin']
            if joint['movable']:
                step = np.eye(4)
                axis = joint['axis'] / np.linalg.norm(joint['axis'])
                step[:3, :3] = Rotation.from_rotvec(angles[joint['name']] * axis).as_matrix()
                transform = transform @ step
            if index == self.base_idx_:
                base_pos = transform[:3, 3].copy()

        # Calculate horizontal distance
        offset = transform[:3, 3] - base_pos
        return float(np.hypot(offset[0], offset[1]))

    def serve_reach(self, request, response):
        """
        Answers a GetTipReach request (fails if pose is unusable).
        """
        if self.joint_chain_ is None:
            response.success = False
            response.message = 'No robot description received yet'
            return response

        # Compute reach per pose
        required = {joint['name'] for joint in self.joint_chain_ if joint['movable']}
        for pose in request.poses:
            angles = dict(zip(pose.name, pose.position))
            missing = required - angles.keys()
            if missing:
                response.reach = []
                response.success = False
                response.message = f'Pose is missing joints: {", ".join(sorted(missing))}'
                return response
            
            response.reach.append(self.get_reach(angles))

        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TipReachServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
