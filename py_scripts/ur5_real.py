import urx
from math import *
robot = urx.Robot("192.168.1.105")
pose = robot.getl()
print(pose)
robot.movel((0.5, -0.3, 0.3, pi/2, -pi, pi), acc=0.1, vel = 0.5)