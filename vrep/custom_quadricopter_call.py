import sim
import sys
sim.simxFinish(-1) 
clientID = sim.simxStart('127.0.0.1',19997,True,True,5000,5)

while clientID != -1:

    # returnCode = sim.simxSynchronous(clientID,True)

    err, copter = sim.simxGetObjectHandle(clientID, "Quadricopter_base", sim.simx_opmode_oneshot_wait)
    err, target = sim.simxGetObjectHandle(clientID, "Quadricopter_target", sim.simx_opmode_oneshot_wait)
    
    target_pos = sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_oneshot)

    ''' 1. gripper signal (yes/no-1/0)
        2. gripper angle (in radians)
        3. yaw angle (in radians)
    '''

    sim.simxSetIntegerSignal(clientID,'actuateGripperSignal',0,sim.simx_opmode_oneshot)
    sim.simxSetIntegerSignal(clientID,'gripperAngle',0,sim.simx_opmode_oneshot)
    sim.simxSetFloatSignal(clientID,'yawAngle',3.14/8,sim.simx_opmode_oneshot)
    # target_pos = sim.simxSetObjectPosition(clientID, target, -1, [1, 0, 0.5], sim.simx_opmode_oneshot)

    sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot)
    err, lin, ang = sim.simxGetObjectVelocity(clientID, copter, sim.simx_opmode_oneshot )
    
else:
    print ('Connection failed.')
    sys.exit('Could not connect.')