import urx
import numpy as np
from collections import namedtuple

# pose = robot.getl()
# print(pose)
# robot.movel((0.5, -0.3, 0.3, pi/2, -pi, pi), acc=0.1, vel = 0.5)

DEFAULT_ORIENT = (0, np.pi, np.pi)
GRIPPER_DEGREE = 0.45

class UR5(object):
    initial_ee_pos = np.asarray((0.5, 0.0, 0.7))
    initial_orient_values = np.asarray(DEFAULT_ORIENT)
    position_bounds = np.asarray(((0.2, 0.7), (-0.3, 0.3), (0.63, 0.9)))
    orientation_bounds = np.asarray(((-np.pi, np.pi), (-np.pi, np.pi), (-np.pi, np.pi)))

    def __init__(self, operating_mode, headless=False, gripper_state=True):
        '''
        Initialize the environment. Connect to the physics server and load the robot and object.
        @param operating_mode: operating mode of the robot. Can be (ee3dof, ee6dof, joints)
        @param headless: if True, run the simulation without GUI
        '''
        assert operating_mode in ('ee3dof', 'ee6dof', 'joints')
        self.operating_mode = operating_mode
        self.DEFAULT_GRIPPER_STATE = gripper_state
        self._connect_robot()
    
    def _observation(self):
        '''
        Compute the observation of the simulation state
        @return: observation of the current state
        '''
        # gripper state
        obs = namedtuple('obs', ["gripper_pos", "gripper_orient", "gripper_vel", "gripper_ang_vel", "obj_pos", "obj_orient", "gripper_state"])
        state = self._robot.getl()
        print("x: ", state[0], "y: ", state[1], "z: ", state[2], "rx: ", state[3], "ry: ", state[4], "rz: ", state[5])
        print(state)
        return obs

    def _connect_robot(self):
        self._robot = urx.Robot("192.168.19.2")
        self._robot.secmon.setDaemon(True)

    def __del__(self):
        '''
        Object destructor. Disconnect from robot.
        '''
        self._robot.close()

if __name__ == "__main__":
    ur5 = UR5('ee3dof')
    try:
        while True:
            obs = ur5._observation()
            print(obs)
    except KeyboardInterrupt:
        pass