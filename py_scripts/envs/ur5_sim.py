"""
UR5 robot arm Environment with default parameters
License: MIT- license
"""

from sys import flags
import pybullet
import pybullet_data
import numpy as np
from collections import namedtuple

class UR5(object):
    initial_ee_pos = np.asarray((0.5, 0.0, 0.8))
    initial_orient_values = np.asarray((0, np.pi/2, np.pi))
    position_bounds = np.asarray(((0.2, 0.7), (-0.3, 0.3), (0.63, 0.9)))
    joint_type = ['REVOLUTE', 'PRISMATIC', 'SPHERICAL', 'PLANAR', 'FIXED']
    models_path = '/Users/sav/rl_research/models'

    def __init__(self, is_train=False):
        self._connect_bullet(is_train)
        pybullet.resetDebugVisualizerCamera(1.4, 45, -30, (0.6, 0, 0.6))
        self._init_world()

    def _connect_bullet(self, is_train):
        if is_train:
            pybullet.connect(pybullet.DIRECT)
        else:
            pybullet.connect(pybullet.GUI)

    def __del__(self):
        pybullet.disconnect()

    def _step_sim(self, action):
        if action is None:
            pybullet.stepSimulation()
            return
        action = np.clip(action, -1, 1)
        new_ee_pos = np.clip(np.array(action) * 0.1 + self._current_ee_pos, self.position_bounds[:, 0], self.position_bounds[:, 1])
        self._move_hand(new_ee_pos)
        pybullet.stepSimulation()
        self._current_ee_pos = new_ee_pos
        
    def _observation(self):
        # gripper state
        obs = namedtuple('obs', ["gripper_pos", "gripper_orient", "gripper_vel", "gripper_ang_vel", "obj_pos", "obj_orient"])
        gripper_position, gripper_orientation, _, _, _, _, gripper_velocity, gripper_angular_velocity = \
            pybullet.getLinkState(self.robot, linkIndex = 7, computeLinkVelocity=True)    
        obs.gripper_pos = np.asarray(gripper_position)
        obs.gripper_orient = pybullet.getEulerFromQuaternion(gripper_orientation)
        obs.gripper_vel = np.asarray(gripper_velocity)
        obs.gripper_ang_vel = np.asarray(gripper_angular_velocity)
        # obs.gripper_closed = self._gripper_state

        # object state
        object_position, object_orientation = pybullet.getBasePositionAndOrientation(self.object)
        obs.obj_pos = np.asarray(object_position)
        obs.obj_orient = pybullet.getEulerFromQuaternion(object_orientation)
        return obs

    def _move_hand(self, target_position, orientation = (0, np.pi/2, np.pi)):
        ee_index = 7
        joint_poses = pybullet.calculateInverseKinematics(
            self.robot,
            ee_index,
            target_position,
            targetOrientation = pybullet.getQuaternionFromEuler(orientation),
            maxNumIterations=1000,
            residualThreshold=.01
        )
        for joint, pos in zip(self.joints, joint_poses):
            pybullet.setJointMotorControl2(
                self.robot, joint['jointID'],
                pybullet.POSITION_CONTROL,
                targetPosition=pos,
            )
        # if not gripper_value is None:
        #     self._gripper_grasping(gripper_value)

    # def _gripper_grasping(self, state):
    #     self._gripper_state = state
    #     value = self.degreeOfGripperClosing
    #     jointPosesGripper = [value - value / 2.,
    #             0,
    #             0,
    #             0,
    #             value - value / 2.,
    #             value - value / 2.,
    #             0,
    #             0,
    #             0,
    #             value - value / 2.
    #             ] if state else [0] * 10
    #     force = 10000
    #     pybullet.setJointMotorControlArray(self.robot, range(9, 19), pybullet.POSITION_CONTROL, jointPosesGripper, forces=[force, force, force, force, force, force, force, force, force, force])

    # def _contactPoints(self):
    #     contact_points = pybullet.getContactPoints(self.object, self.robot)
    #     return len(contact_points)
        
    def _reset_world(self):
        x_pos_obj = np.random.uniform(*self.position_bounds[0])
        y_pos_obj = np.random.uniform(*self.position_bounds[1])
        z_pos_obj = np.random.uniform(*self.position_bounds[2])
        self.obj_origin_pos = np.asarray((x_pos_obj, y_pos_obj, z_pos_obj))
        pybullet.resetBasePositionAndOrientation(
            self.object, self.obj_origin_pos, pybullet.getQuaternionFromEuler((0, 0, 0)))
        for joint in self.joints:
            pybullet.resetJointState(
                self.robot, joint['jointID'],
                0,
                0
            )
        pybullet.stepSimulation()
        self._move_hand(self.initial_ee_pos, self.initial_orient_values)
        # step simulation to let the robot settle
        for _ in range(100):
            pybullet.stepSimulation()
        self._current_ee_pos = np.asarray(self.initial_ee_pos)
        pybullet.stepSimulation()

    def _init_world(self):
        pybullet.resetSimulation()
        pybullet.setGravity(0, 0, -9.8)
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.planeId = pybullet.loadURDF('plane.urdf')
        self.table = pybullet.loadURDF('table/table.urdf', globalScaling=1, basePosition=[0.5, 0, 0])
        pybullet.setAdditionalSearchPath(self.models_path)
        self.object = pybullet.loadURDF('sphere/sphere.urdf', (0.6, 0, 0.8), pybullet.getQuaternionFromEuler([0, 0, 0]))
        self.robot, self.joints, self.joints_rev, self.joints_fix = self._spawn_robot('ur5/ur5.urdf', pos=(0, 0, 0.63), angle=(0, 0, 0), )        
        self._reset_world()

    def _spawn_robot(self, urdf_path, pos, angle, ee_pos = None, ee_orientation = None):
        robot = pybullet.loadURDF(urdf_path, pos, pybullet.getQuaternionFromEuler(angle))
        joints = []
        joints_rev = {}
        joints_fix = {}

        for joint_id in range(pybullet.getNumJoints(robot)):
            info = pybullet.getJointInfo(robot, joint_id)
            data = {
                'jointID': info[0],
                'jointName': info[1].decode('utf-8'),
                'jointType': self.joint_type[info[2]],
                'jointLowerLimit': info[8],
                'jointUpperLimit': info[9],
                'jointMaxForce': info[10],
                'jointMaxVelocity': info[11]
            }
            if data['jointType'] != 'FIXED':
                joints.append(data)
                joints_rev[data['jointName']] = joint_id
            else:
                joints_fix[data['jointName']] = joint_id
        return robot, joints, joints_rev, joints_fix