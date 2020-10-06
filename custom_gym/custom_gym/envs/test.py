import env
import sim

try:
	sim.simxFinish(-1)
	clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
	env = env.GripperEnv(clientID)
	state, action = env.reset()

	if clientID != -1:
		print('Connected to remote API server', clientID)	
		env.init_sensors(clientID)
		# env.start_sim()

		while  sim.simxGetConnectionId(clientID) != -1:
			env.rotor_data = [5.4,5.4,5.4,5.4]
			state, done, action = env.step(action, True) # true or false value to be determined from the proximity sensor
			if done == True:
				env.stop_sim()
	else:
		print("Failed to connect to remote API")
		env.stop_sim()
		sim.simxFinish(clientID)

except KeyboardInterrupt:
	env.stop_sim()
	sim.simxFinish(clientID)
