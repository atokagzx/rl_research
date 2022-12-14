"""
UR5 robot arm Environment with default parameters
License: MIT- license
"""

import os
import pybullet
import pybullet_data
import numpy as np
from collections import namedtuple
from time import sleep
JOINT_TYPE = ('REVOLUTE', 'PRISMATIC', 'SPHERICAL', 'PLANAR', 'FIXED')
Joint = namedtuple('joint', ["index", "name", "type",  "linkName", "lowerLimit", "upperLimit", "maxForce", "maxVelocity","damping", "friction"])

DEFAULT_ORIENT = (0, np.pi, np.pi)
GRIPPER_DEGREE = 0.45

class UR5(object):
    initial_ee_pos = np.asarray((0.5, 0.0, 0.7))
    initial_orient_values = np.asarray(DEFAULT_ORIENT)
    position_bounds = np.asarray(((0.2, 0.7), (-0.3, 0.3), (0.63, 0.9)))
    orientation_bounds = np.asarray(((-np.pi, np.pi), (-np.pi, np.pi), (-np.pi, np.pi)))
    models_path = str(os.path.join(os.path.dirname(__file__), '..', 'models'))
    
    def __init__(self, operating_mode, headless=False, gripper_state=True):
        '''
        Initialize the environment. Connect to the physics server and load the robot and object.
        @param operating_mode: operating mode of the robot. Can be (ee3dof, ee6dof, joints)
        @param headless: if True, run the simulation without GUI
        '''
        assert operating_mode in ('ee3dof', 'ee6dof', 'joints')
        self.operating_mode = operating_mode
        self.ee_index = 9
        self.DEFAULT_GRIPPER_STATE = gripper_state
        self._connect_bullet(headless)
        pybullet.resetDebugVisualizerCamera(1.4, 45, -30, (0.6, 0, 0.6))
        self._init_world()

    def _connect_bullet(self, headless):
        '''
        Connect to the physics server
        @param headless: if True, run the simulation without GUI
        '''
        if headless:
            pybullet.connect(pybullet.DIRECT)
        else:
            pybullet.connect(pybullet.GUI)

    def __del__(self):
        '''
        Object destructor. Disconnect from the physics server.
        '''
        pybullet.disconnect()

    def _step_sim(self, action):
        '''
        Step the simulation forward
        @param action: action to be applied to the robot
        '''
        if action is None:
            pybullet.stepSimulation()
            return
        action = action.copy()
        action = np.clip(action, -1, 1)
        gripper_state = self.DEFAULT_GRIPPER_STATE
        gripper_position, gripper_orientation, _, _, _, _, _gripper_velocity, _gripper_angular_velocity = \
            pybullet.getLinkState(self.robot, linkIndex = self.ee_index, computeLinkVelocity=True)
        gripper_orientation = pybullet.getEulerFromQuaternion(gripper_orientation)
        if self.operating_mode in 'ee3dof':
            assert len(action) in (3, 4)
            if len(action) == 4:
                gripper_state = action[3]
            pos = action[:3]
            orientation = DEFAULT_ORIENT
            pos = np.clip(np.array(pos) * 0.2 + gripper_position, self.position_bounds[:, 0], self.position_bounds[:, 1])

        elif self.operating_mode == 'ee6dof':
            assert len(action) in (6, 7)
            if len(action) == 7:
                gripper_state = action[6]
            pos = action[:3]
            orientation = action[3:6]
            pos = np.clip(np.array(pos) * 0.2 + gripper_position, self.position_bounds[:, 0], self.position_bounds[:, 1])
            orientation = np.clip(np.array(orientation) * 0.5 + gripper_orientation, self.orientation_bounds[:, 0], self.orientation_bounds[:, 1])

        elif self.operating_mode == 'joints':
            raise NotImplementedError
            assert len(action) in (6, 7)
            if len(action) == 7:
                gripper_state = action[6]
            joints = action[:6]

        if self.operating_mode in ('ee3dof', 'ee6dof'):
            self._move_hand(pos, orientation, gripper_state = gripper_state)

        pybullet.stepSimulation()
        
    def _observation(self):
        '''
        Compute the observation of the simulation state
        @return: observation of the current state
        '''
        # gripper state
        obs = namedtuple('obs', ["gripper_pos", "gripper_orient", "gripper_vel", "gripper_ang_vel", "obj_pos", "obj_orient", "gripper_state"])
        gripper_position, gripper_orientation, _, _, _, _, gripper_velocity, gripper_angular_velocity = \
            pybullet.getLinkState(self.robot, linkIndex = self.ee_index, computeLinkVelocity=True)    
        obs.gripper_pos = np.asarray(gripper_position)
        obs.gripper_orient = pybullet.getEulerFromQuaternion(gripper_orientation)
        obs.gripper_vel = np.asarray(gripper_velocity)
        obs.gripper_ang_vel = np.asarray(gripper_angular_velocity)
        obs.gripper_state = pybullet.getJointState(self.robot, self.joints_revolute[6].index)[0] / GRIPPER_DEGREE > 0.5
        # object state
        object_position, object_orientation = pybullet.getBasePositionAndOrientation(self.object)
        obs.obj_pos = np.asarray(object_position)
        obs.obj_orient = pybullet.getEulerFromQuaternion(object_orientation)
        return obs

    def _move_hand(self, target_position, orientation, gripper_state):
        '''
        Move the robot's end effector to the target position.
        Calculate the inverse kinematics and set the joint positions.
        @param target_position: target position of the end effector
        @param orientation: target orientation of the end effector
        '''
        joint_poses = pybullet.calculateInverseKinematics(
            self.robot,
            self.ee_index,
            target_position,
            targetOrientation = pybullet.getQuaternionFromEuler(orientation),
            maxNumIterations=1000,
            residualThreshold=.01
        )
        for joint, pos in zip(self.joints_revolute[:6], joint_poses):
            pybullet.setJointMotorControl2(
                self.robot, joint.index,
                pybullet.POSITION_CONTROL,
                targetPosition=pos,
            )
        self._gripper_grasping(gripper_state)

    def _gripper_grasping(self, state, force = 1000):
        '''
        Control the gripper
        @param state: if True, close the gripper
        @param force: force to be applied to the gripper
        '''
        value = GRIPPER_DEGREE if state else 0
        jointPosesGripper = [value,
                0,
                value,
                value,
                0,
                value,
                ] 
        for joint, pos in zip(self.joints_revolute[6:], jointPosesGripper):
            pybullet.setJointMotorControl2(
                self.robot, joint.index,
                pybullet.POSITION_CONTROL,
                targetPosition=pos,
                force=force
            )

    def _reset_world(self):
        '''
        Reset the world to initial state with random object position.
        Should be called at the beginning of each episode
        '''
        # reset object position
        x_pos_obj = np.random.uniform(0.3, 0.6)
        y_pos_obj = np.random.uniform(-0.25, 0.25)
        z_pos_obj = 0.65
        self.obj_origin_pos = np.asarray((x_pos_obj, y_pos_obj, z_pos_obj))
        pybullet.resetBasePositionAndOrientation(
            self.object, self.obj_origin_pos, pybullet.getQuaternionFromEuler((0, 0, 0)))

        # reset robot's end effector position
        for joint, pos in zip(self.joints_revolute, (0, -np.pi/2, np.pi/2, 0, 0, 0)):
            pybullet.resetJointState(self.robot, joint.index, pos, 0)

        pybullet.stepSimulation()
        self._move_hand(self.initial_ee_pos, self.initial_orient_values, self.DEFAULT_GRIPPER_STATE)
        # step simulation to let the robot settle
        for _ in range(100):
            pybullet.stepSimulation()

    def _init_world(self):
        '''
        Initialize the world with the robot and the object
        '''
        # load plane from pybullet_data objects package
        pybullet.resetSimulation()
        pybullet.setGravity(0, 0, -9.8)
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.planeId = pybullet.loadURDF('plane.urdf')
        self.table = pybullet.loadURDF('table/table.urdf', globalScaling=1, basePosition=[0.5, 0, 0])

        # set user path for models and load robot
        self.object = pybullet.loadURDF('cube_small.urdf', (0.7, 0, 0.65), pybullet.getQuaternionFromEuler([0, 0, 0]), globalScaling = 0.8)  
        pybullet.setAdditionalSearchPath(self.models_path)
        self._spawn_robot('ur5/ur5_gripper.urdf')

    def _spawn_robot(self, urdf_path, pos=(0, 0, 0.63), angle=(0, 0, 0)):
        '''
        Spawn robot from urdf file and get joints info
        @param urdf_path: path to urdf file
        @param pos: robot's base position
        @param angle: orientation of robot's base
        '''
        robot = pybullet.loadURDF(urdf_path, pos, pybullet.getQuaternionFromEuler(angle))
        joints = []
        for joint_id in range(pybullet.getNumJoints(robot)):
            info = pybullet.getJointInfo(robot, joint_id)
            joint = Joint(info[0], info[1].decode("utf-8"), JOINT_TYPE[info[2]], info[12].decode("utf-8"), info[8], info[9], info[10], info[11], info[6], info[7])
            joints.append(joint)
        self.joints = joints
        self.joints_fixed = tuple(filter(lambda x: x.type == 'FIXED', joints))
        self.joints_revolute = tuple(filter(lambda x: x.type == 'REVOLUTE', joints))
        print(*self.joints_fixed, sep="\n")
        print(*self.joints_revolute, sep="\n")
        self.robot = robot