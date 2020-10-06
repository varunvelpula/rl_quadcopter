import gym
from gym import error, spaces, utils
from gym.utils import seeding
import sim
import numpy as np
import time
import math
import pdb
import time

class RLQuad(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self):
    # self.min_position = -1
    # self.max_position = 10
    self.steps = 0
    self.goal = [0.5,0.5,0.2]
    self.start = [0,0,1]
    self.action = spaces.Discrete(6)
    # sim.simxFinish(-1)
    self.clientID = sim.simxStart('127.0.0.1',19997,True,False,5000,5)
    err, self.copter = sim.simxGetObjectHandle(self.clientID, "Quadricopter_base", sim.simx_opmode_oneshot_wait)
    err, self.target = sim.simxGetObjectHandle(self.clientID, "Quadricopter_target", sim.simx_opmode_oneshot_wait)
    err, self.sphere = sim.simxGetObjectHandle(self.clientID, "Sphere", sim.simx_opmode_oneshot_wait)
    err, self.joint_6 = sim.simxGetObjectHandle(self.clientID, "Fourbar_joint6", sim.simx_opmode_oneshot_wait)
    sim.simxStartSimulation(self.clientID,sim.simx_opmode_oneshot)
    # self.observation_space = spaces.Box(
    #   low = np.array([self.min_position,self.min_position,0],
    #                     dtype = np.float32)
    #   high = np.array([self.max_position,self.max_position,self.max_position],
    #                     dtype = np.float32)
    #   dtype = np.float32
    # )

    self.observation = spaces.Tuple((
                spaces.Discrete(5),
                spaces.Discrete(5),
                spaces.Discrete(10),
                spaces.Discrete(2)
                ))
    self.moves = {  # moves of the quadcopter defined interms of the grid coordinates
        0: (1,0,0),
        1: (-1,0,0),
        2: (0,1,0),
        3: (0,-1,0),
        4: (0,0,1),
        5: (0,0,-1),
      }
    self.S = [0,0,9]
    self.reset()
    
  
  def step(self, action):
    if action <=5:
      x,y,z = self.moves[action]
      self.S = [self.S[0] + x, self.S[1] + y, self.S[2] + z]
      if self.S[0]<0 or self.S[1]<0 or self.S[2]<0:
        self.S = [max(0, self.S[0]), max(0, self.S[1]), max(0, self.S[2])]
        return self.S,-10,False,{}
      if self.S[0]>=5 or self.S[1]>=5 or self.S[2]>=10:
        self.S = [min(self.S[0], 5 - 1), min(self.S[1], 5 - 1), min(self.S[2], 10 - 1)]
        return self.S,-10,False,{}
      else:
        sim.simxSetIntegerSignal(self.clientID,'actuateGripperSignal',0,sim.simx_opmode_oneshot)
        sim.simxSetObjectPosition(self.clientID, self.target,-1, [0+0.1*self.S[0],0+0.1*self.S[1],0+0.1*self.S[2]],sim.simx_opmode_oneshot)  #starting point is (0.3,0.3,0.3)
        while True:
          distance = sim.simxGetObjectPosition(self.clientID,self.copter,self.target,sim.simx_opmode_oneshot_wait)
          if np.abs(np.linalg.norm(np.array(distance[1])))<0.02:
            time.sleep(2)
            break
        # if (sim.simxGetCollisionHandle(self.clientID,))
        return self.S,-1,False,{}

    elif action==6:
      # pdb.set_trace()
      gripperAngle = 150
      angle_displacement = (gripperAngle/2-45)*math.pi/180
      err, joint_6 = sim.simxGetObjectHandle(self.clientID, "Fourbar_joint6", sim.simx_opmode_oneshot)
      # sim.simxClearIntegerSignal(self.clientID,'actuateGripperSignal',sim.simx_opmode_oneshot)
      # sim.simxClearIntegerSignal(self.clientID,'gripperAngle',sim.simx_opmode_oneshot)
      sim.simxSetIntegerSignal(self.clientID,'actuateGripperSignal',1,sim.simx_opmode_oneshot)
      sim.simxSetIntegerSignal(self.clientID,'gripperAngle',gripperAngle,sim.simx_opmode_oneshot)

      while True:       

        current_joint6_angle = sim.simxGetJointPosition(self.clientID, joint_6, sim.simx_opmode_oneshot)
        if abs(abs(current_joint6_angle[1])-abs(angle_displacement))<0.05*math.pi/180:
          time.sleep(5)
          break
      
      oldPos = sim.simxGetObjectPosition(self.clientID, self.sphere, -1, sim.simx_opmode_oneshot)
      sim.simxSetObjectPosition(self.clientID, self.target, self.copter, [0,0,0.5], sim.simx_opmode_oneshot)

      while True:
        distance = sim.simxGetObjectPosition(self.clientID,self.copter,self.target,sim.simx_opmode_oneshot_wait)
        if np.abs(np.linalg.norm(np.array(distance[1])))<0.02:
          time.sleep(2)
          break
      newPos = sim.simxGetObjectPosition(self.clientID, self.sphere, -1, sim.simx_opmode_oneshot)
      if np.linalg.norm(np.array(newPos[1])-np.array(oldPos[1]))>0.4 and np.linalg.norm(np.array(newPos)-np.array(oldPos))<0.5:
        return self.S,100,True,{}
      else:
        return self.S,-100,True,{}
    
  def reset(self):
    # stop the simulation and restart the simulation
    self.S = [0,0,9]
    # sim.simxStartSimulation(self.clientID,sim.simx_opmode_oneshot)
    
    sim.simxSetObjectPosition(self.clientID,self.target,-1,[0,0,1],sim.simx_opmode_oneshot)
    sim.simxSetObjectPosition(self.clientID,self.sphere,-1,[0,0,0.175],sim.simx_opmode_oneshot)
    while True:
      distance = sim.simxGetObjectPosition(self.clientID,self.copter,self.target,sim.simx_opmode_oneshot_wait)
      sim.simxSetIntegerSignal(self.clientID,'gripperAngle',90,sim.simx_opmode_oneshot)
      sim.simxSetIntegerSignal(self.clientID,'actuateGripperSignal',0,sim.simx_opmode_oneshot)
      angle = sim.simxGetJointPosition(self.clientID,self.joint_6,sim.simx_opmode_oneshot)
      if np.abs(np.linalg.norm(np.array(distance[1])))<0.02 and abs(angle[1])<0.5*math.pi/180:
        sim.simxSetIntegerSignal(self.clientID,'actuateGripperSignal',0,sim.simx_opmode_oneshot)
        time.sleep(5)
        break
    return self.S