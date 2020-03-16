import time
import numpy as np
import math

useNullSpace = 1
ikSolver = 0
pandaEndEffectorIndex = 11 #8
pandaNumDofs = 7

ll = [-7]*pandaNumDofs
#upper limits for null space (todo: set them to proper range)
ul = [7]*pandaNumDofs
#joint ranges for null space (todo: set them to proper range)
jr = [7]*pandaNumDofs
#restposes for null space
jointPositions=[0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02]
rp = jointPositions

class PandaCamSim(object):
  def __init__(self, bullet_client, offset):
    self.bullet_client = bullet_client
    self.offset = np.array(offset)
    self.tray_offset = np.array([0,0,0])
    #print("offset=",offset)
    flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
    legos=[]
    self.bullet_client.loadURDF("tray/traybox.urdf", [0+offset[0], 0+offset[1], -0.6+offset[2]]+self.tray_offset, [-0.5, -0.5, -0.5, 0.5], flags=flags)
    legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([0.1, 0.3, -0.5])+self.offset+self.tray_offset, flags=flags))
    legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([-0.1, 0.3, -0.5])+self.offset+self.tray_offset, flags=flags))
    legos.append(self.bullet_client.loadURDF("lego/lego.urdf",np.array([0.1, 0.3, -0.7])+self.offset+self.tray_offset, flags=flags))
    sphereId = self.bullet_client.loadURDF("sphere_small.urdf",np.array( [0, 0.3, -0.6])+self.offset+self.tray_offset, flags=flags)
    self.bullet_client.loadURDF("sphere_small.urdf",np.array( [0, 0.3, -0.5])+self.offset+self.tray_offset, flags=flags)
    self.bullet_client.loadURDF("sphere_small.urdf",np.array( [0, 0.3, -0.7])+self.offset+self.tray_offset, flags=flags)
    orn=[-0.707107, 0.0, 0.0, 0.707107]#p.getQuaternionFromEuler([-math.pi/2,math.pi/2,0])
    eul = self.bullet_client.getEulerFromQuaternion([-0.5, -0.5, -0.5, 0.5])
    self.panda = self.bullet_client.loadURDF("franka_panda/panda.urdf", np.array([0,0,0])+self.offset, orn, useFixedBase=True, flags=flags)
    print("Total Number of Joints:{}".format(self.bullet_client.getNumJoints(self.panda)))
    index = 0
    for j in range(self.bullet_client.getNumJoints(self.panda)):
      self.bullet_client.changeDynamics(self.panda, j, linearDamping=0, angularDamping=0)
      info = self.bullet_client.getJointInfo(self.panda, j)
  
      jointName = info[1]
      jointType = info[2]
      if (jointType == self.bullet_client.JOINT_PRISMATIC):
        
        self.bullet_client.resetJointState(self.panda, j, jointPositions[index]) 
        index=index+1
      if (jointType == self.bullet_client.JOINT_REVOLUTE):
        self.bullet_client.resetJointState(self.panda, j, jointPositions[index]) 
        index=index+1

    # Set camera Properties
    self.camTargetPos = np.array([0, 0, 0])
    self.camUpVector = [1, 0, 0]
    self.projMat = self.bullet_client.computeProjectionMatrixFOV(
    fov=45.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=3.1)
    self.t = 0.
  def reset(self):
    pass

  def step(self):
    t = self.t
    self.t += 1./60.
    pos = [self.offset[0]+0.2 * math.sin(1.5 * t), self.offset[1]+.044+0.5, self.offset[2]+-0.6 + 0.1 * math.cos(1.5 * t)]
    orn = self.bullet_client.getQuaternionFromEuler([math.pi/2.,0.,0.])
    jointPoses = self.bullet_client.calculateInverseKinematics(self.panda,pandaEndEffectorIndex, pos, orn, ll, ul,
      jr, rp, maxNumIterations=5)
    for i in range(pandaNumDofs):
        self.bullet_client.setJointMotorControl2(self.panda, i, self.bullet_client.POSITION_CONTROL, jointPoses[i],force=5 * 240.)

    #Set up Camera view
    endPos, endOrn,localInertialFramePosition, localInertialFrameOrientation,worldLinkFramePosition, worldLinkFrameOrientation= self.bullet_client.getLinkState(self.panda, 7)
    rotMat = np.array(self.bullet_client.getMatrixFromQuaternion(endOrn)).reshape((3,3))
    camPos = np.array(endPos)+np.dot(rotMat,np.array([0,0.,.1])) 
    targetPos = np.array(endPos)+np.dot(rotMat, np.array([0,0,0.5]))
    viewMatrix = self.bullet_client.computeViewMatrix(cameraEyePosition=camPos, cameraTargetPosition=targetPos, cameraUpVector = self.camUpVector)
    width, height, rgbImg, depthImg, segImg = self.bullet_client.getCameraImage(width=224, height=224, viewMatrix=viewMatrix,projectionMatrix=self.projMat)
    pass
  
