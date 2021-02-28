import os
import time
import math
import numpy as np
import pybullet as p
import pybullet_data
import pbsm
from pbsm import UR5_2F85, UR5, Robotiq2F85

cid = p.connect(p.GUI)#or p.DIRECT for non-graphical version

print('Current Simulation Parameters:')
print(p.getPhysicsEngineParameters(cid))

#The fixedTimeStep and numSolverIterations are the most important parameters to trade-off quality versus performance
fixedTimeStep = 1. / 240  #Default is 1./240
numSolverIterations = 500 #Default is 50

#Its best to keep default values for simulation parameters.
p.setPhysicsEngineParameter(numSolverIterations=numSolverIterations)
#"Warning: in many cases it is best to leave the timeStep to default, which is 240Hz. Several
#parameters are tuned with this value in mind. For example the number of solver iterations and
#the error reduction parameters (erp) for contact, friction and non-contact joints are related to the
#time step. If you change the time step, you may need to re-tune those values accordingly,
#especially the erp values." -- The Manual
p.setTimeStep(fixedTimeStep)
p.setRealTimeSimulation(0)

#Only one additional search path can be supplied to PyBullet
p.setAdditionalSearchPath(pybullet_data.getDataPath())
print("PyBullet data path: ")
print(pybullet_data.getDataPath())

#Disable rendering during loading makes it much faster
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

#Instanciate the UR5 only
#arm = UR5(cid)
#or instanciate the 2F85 only
#gripper = Robotiq2F85(cid, startingOrientationRAD=[np.deg2rad(-90),0,0])
#or instanciate a UR5 equipped with a 2F85
robot = UR5_2F85(cid)

#You might want to play with these values
#but use reasonable values.
robot.setGripperMaxForce(10)
robot.setArmMaxVelocity(0.25)
robot.setArmMaxForce(150)

#Initialize joints position
initial_joints_positions_degrees = [0, -90, 45, -45, -90, 0, 0, 0]
robot.initJoints(initial_joints_positions_degrees)

#Re-enable rendering
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

#Set the downward gravity acceleration
p.setGravity(0,0,-9.81)

#Add the plane only after having initialized the position of the robot arm since moving the robot arm to its initial position can cause a collision
#between the plane and the robot arm.
planeId = p.loadURDF("plane.urdf")

#A wild hammer enters the scene...
hammer_model_path = os.path.dirname(pbsm.__file__) + "/models/HammerSmaller/urdf/HammerSmaller.urdf"
hammerId = p.loadURDF(hammer_model_path, [0.35,0,0.03], p.getQuaternionFromEuler([0,math.radians(90),math.radians(180)]))
p.changeDynamics(hammerId,-1,lateralFriction=1, spinningFriction=1)

#Perform a sequence
robot.addGoal([(0.45,0,0.1),p.getQuaternionFromEuler([0,np.deg2rad(-180),0]),0,0])       #Above the hammer
robot.addGoal([(0.45,0,0.02),p.getQuaternionFromEuler([0,np.deg2rad(-180),0]),0,0])      #Pre-grasp
robot.addGoal([(0.45,0,0.02),p.getQuaternionFromEuler([0,np.deg2rad(-180),0]),83,83])    #Grasp
robot.addGoal([(0.45,0,0.2),p.getQuaternionFromEuler([0,np.deg2rad(-180),0]),83,83])     #Lift up
robot.addGoal([(0,0,0,0,np.deg2rad(-30),0),(0,0)])  # Move wrist (Relative joint movement)
robot.addGoal([(0,0,0,0,np.deg2rad(+60),0),(0,0)])  # Move wrist (Relative joint movement)
robot.addGoal([(0,0,0,0,np.deg2rad(-30),0),(0,0)])  # Move wrist (Relative joint movement)
robot.addGoal([(0,0,0,np.deg2rad(-30),0,0),(0,0)])  # Move wrist (Relative joint movement)
robot.addGoal([(0,0,0,np.deg2rad(+60),0,0),(0,0)])  # Move wrist (Relative joint movement)
robot.addGoal([(0,0,0,np.deg2rad(-30),0,0),(0,0)])  # Move wrist (Relative joint movement)


#Run the simulator
for i in range(8000):    
    p.stepSimulation()
    robot.setCurrentGoal()
    time.sleep(fixedTimeStep)

#Disconnect from the physics server
p.disconnect()
