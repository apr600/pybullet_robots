import pybullet as p
import pybullet_data as pd
import math
import time
import numpy as np
import panda_cam


def move_circle(t, offset, robot):
    pos = [offset[0]+0.2 * math.sin(1.5 * t), offset[1]+.044+0.5, offset[2]+-0.6 + 0.1 * math.cos(1.5 * t)]
    orn = robot.bullet_client.getQuaternionFromEuler([math.pi/2.,0.,0.])
    return pos, orn

# Set up Frank Robot Env
timeStep=1./60.
offset = [0.,0.,0.]
panda = panda_cam.PandaCamSim(render=True, ts=timeStep,offset=offset)
t = 0.

while (1):
    t += timeStep
    pos, orn = move_circle(t,offset,panda)
    panda.step(pos, orn)
    time.sleep(timeStep)
