import sim
import sys
sim.simxFinish(-1) 
clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5)

if clientID != -1:
    print ('Connected to remote API server.')

    returnCode = sim.simxSynchronous(clientID,True)

    err, copter = sim.simxGetObjectHandle(clientID, "Quadricopter_base", sim.simx_opmode_oneshot_wait)
    err, target = sim.simxGetObjectHandle(clientID, "Quadricopter_target", sim.simx_opmode_oneshot_wait)
    
    target_pos = sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_oneshot)
    print(target_pos)
      
    target_pos = sim.simxSetObjectPosition(clientID, target, -1, [1, 0, 0.5], sim.simx_opmode_oneshot)

    sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot_wait)
    err, lin, ang = sim.simxGetObjectVelocity(clientID, copter, sim.simx_opmode_oneshot )
    print(target_pos)
    print(lin)
    
else:
    print ('Connection failed.')
    sys.exit('Could not connect.')