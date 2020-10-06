import numpy as np
import torch
import math
import torch as T 
import torch.nn as nn
import torch.nn.functional as F 
import torch.optim as optim
from torch.autograd import Variable
import sim
import sim_rotors
from scipy.stats import norm
import sim_imu

class GripperEnv(object):
	def __init__(self, clientID):
		self.clientID = clientID
		self.goal_velocity = 0.2
		self.min_action = 0
		self.max_action = 1
		self.min_position = 0
		self.max_position = 100
		self.initial_position = [0,0,0]
		self.rotor_data = [0,0,0,0]
		self.goal_position = [2,1,1]
		err, self.quadHandle = sim.simxGetObjectHandle(self.clientID, "Quadricopter_base", sim.simx_opmode_blocking)
		err, target = sim.simxGetObjectHandle(self.clientID, "Sphere", sim.simx_opmode_oneshot_wait)
		target_pos = sim.simxSetObjectPosition(self.clientID, target, -1, self.goal_position, sim.simx_opmode_oneshot)
		self.low_state = np.array([self.min_position, self.min_position, self.min_position], dtype = np.float32)
		self.max_state = np.array([self.max_position, self.max_position, self.max_position], dtype = np.float32)

	def init_sensors(self, clientID):
		self.clientID = clientID

		# initialize proximity sensors
		err, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(self.clientID, self.quadHandle, sim.simx_opmode_streaming)

		# initialize rotors
		sim_rotors.init_rotors(self.clientID)

		# reset quadrotor position
		sim.simxSetObjectPosition(self.clientID, self.quadHandle, -1, self.initial_position, sim.simx_opmode_oneshot)

	def start_sim(self):
		sim.simxStartSimulation(self.clientID, sim.simx_opmode_oneshot_wait)
		return

	def stop_sim(self):
		sim.simxStopSimulation(self.clientID, sim.simx_opmode_oneshot_wait)
		return

	def step(self, action, object_presence):

		# getting the current state of the quadcopter
		state = sim.simxGetObjectPosition(self.clientID, self.quadHandle, -1, sim.simx_opmode_oneshot)
		state = np.array(state)
		state = state[1]
		self.do_action()

		# checking the waypoint
		err, waypt = sim.simxGetObjectHandle(self.clientID, "Quadricopter_target", sim.simx_opmode_oneshot_wait)
		waypt_position = sim.simxGetObjectPosition(self.clientID, waypt, -1, sim.simx_opmode_oneshot)  

		# calculating the waypoint
		while waypt_position != self.goal_position:
			if state[0] != self.goal_position[0] and state[1] != self.goal_position[1] and state[2] != self.goal_position[2]:
				state[0] = state[0] + (1-action)*(self.goal_position[0]-state[0])*self.goal_velocity
				state[1] = state[1] + (1-action)*(self.goal_position[1]-state[1])*self.goal_velocity
				state[2] = state[2] + (1-action)*(self.goal_position[2]-state[2])*self.goal_velocity
				state[0] = np.clip(state[0], self.min_position, self.max_position)
				state[1] = np.clip(state[1], self.min_position, self.max_position)
				state[2] = np.clip(state[2], self.min_position, self.max_position)

				# assigning the next waypoint
				sim.simxSetObjectPosition(self.clientID, waypt, -1, state, sim.simx_opmode_oneshot)
				sim.simxStartSimulation(self.clientID, sim.simx_opmode_oneshot_wait)
				# print(state)

		done = bool(abs(state[0] - self.goal_position[0]) <= 0.001 and abs(state[1] - self.goal_position[1]) <= 0.001 and abs(state[2] - self.goal_position[2]) <= 0.001)

		if object_presence == True and done == True:
			action = self.max_action
			print('Object present', action)

		if object_presence == False and done == True:
			action = self.min_action
			print('object absent', action)

		return np.array(state), done, action

	def do_action(self):
		sim_rotors.move_rotors(self.clientID, self.rotor_data)
		return

	def reset(self):
		state = [0]*3
		state[0] = self.initial_position[0]
		state[1] = self.initial_position[1]
		state[2] = self.initial_position[2]
		action = self.min_action

		return np.array(state), action
