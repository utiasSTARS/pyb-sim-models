import pbsm
from pbsm import UR5,CompositeTestObject
import numpy as np
import os
import time
import pybullet as p
import pybullet_data


#Dictionnary of interesting configurations
# {Name: [Steel_weights_positions, Plastic_weights_positions]}
configurations = {
    "Hammer":       [[0,1,2],               [3,4,5,6,8]],
    "Barbell":      [[0,1,2,7,8,9],         [3,4,5,6]],
    "Tee":          [[0,1,2,3,4,5,6],       []],
    "Homogeneous":  [[0,1,2,3,4,5,6,7,8,9], []],
    "Corners":      [[0,2,7,9],             []],
    "Rod":          [[1,3,4,5,6,8],         []],
    "Half-N-Half":  [[0,1,2,3,4],           [5,6,7,8,9]]
}

CHOSEN_CONFIG   = "Hammer"
steel_weights   = configurations[CHOSEN_CONFIG][0]
plastic_weights = configurations[CHOSEN_CONFIG][1]

#The orientation and position of the object relative to the end-effector of the robot depends
#on which bolts are used to attach the object onto the flange of the robot.
#There are ten possibilities, each centering a weight hole with the end-effector. 
mm_to_m    = 1e-3
weight_pos = mm_to_m*np.array([[65,125,25],[0,125,25],[-65,125,25],[0,75,25],[0,25,25],[0,-25,25],[0,-75,25],[0,-125,25],[-65,-125,25],[65,-125,25]])
#The contact point is on the surface of the object (z=0), not at its center (z=25).
weight_pos[:,2] = np.zeros((10,))

from math import radians
anchors = {
    "1-2":   [[1, 2],  weight_pos[0], [0,radians(90),0]],
    "0-5":   [[0, 5],  weight_pos[1], [0,radians(90),0]],
    "3-4":   [[3, 4],  weight_pos[2], [0,radians(90),0]],
    "5-6":   [[5, 6],  weight_pos[3], [0,radians(90),0]],
    "6-7":   [[6, 7],  weight_pos[4], [0,radians(90),0]],
    "7-8":   [[7, 8],  weight_pos[5], [0,radians(90),0]],
    "8-9":   [[8, 9],  weight_pos[6], [0,radians(90),0]],
    "10-11": [[10,11], weight_pos[7], [0,radians(90),0]],
    "9-14":  [[9, 14], weight_pos[8], [0,radians(90),0]],
    "12-13": [[12,13], weight_pos[9], [0,radians(90),0]]
}

CHOSEN_ANCHOR      = "7-8"
anchoring_bolts    = anchors[CHOSEN_ANCHOR][0]
object_pos_wrt_ee  = anchors[CHOSEN_ANCHOR][1]
object_rpy_wrt_ee  = anchors[CHOSEN_ANCHOR][2]

#More bolts can be added to hold the object together, if needed.
extra_bolts = [0,14]
bolts_attached = anchoring_bolts + extra_bolts

#Generates the XACRO output file
PBSM_PATH = os.path.dirname(pbsm.__file__)
output_xacro_path = PBSM_PATH+'/models/CompositeTestObject/object.xacro'
CompositeTestObject(bolts_attached, steel_weights, plastic_weights, output_xacro_path, print_params=False, linkxacro=True)

#Read resulting XACRO file
obj_xacro =  open(output_xacro_path, 'r')
content = obj_xacro.read()
#Modify paths such that the main XACRO can reach the meshes
content = content.replace('filename="','filename="./CompositeTestObject/')

#Write main XACRO file
main_xacro_content = \
'''<?xml version="1.0"?>
<robot name="UR5_compositeobject" xmlns:xacro="http://ros.org/wiki/xacro" >

	<!-- On the first day, Darwin created the world -->
    <link name="world"/>

	<!-- Add UR5 model to our world -->
    <xacro:include filename="./ur5/ur5.urdf.xacro" />
	
	<!-- Attach UR5's base to the ground so it doesnt fall over -->
    <joint name="fixed" type="fixed">
            <parent link="world"/>
            <child link="base_link"/>
    </joint>
	
	<!-- Instanciate a UR5 -->
	<xacro:ur5_robot prefix="" joint_limited="true"
    		shoulder_pan_lower_limit="${{-2*pi}}" shoulder_pan_upper_limit="${{2*pi}}"
    		shoulder_lift_lower_limit="${{-2*pi}}" shoulder_lift_upper_limit="${{2*pi}}"
    		elbow_joint_lower_limit="${{-2*pi}}" elbow_joint_upper_limit="${{2*pi}}"
    		wrist_1_lower_limit="${{-2*pi}}" wrist_1_upper_limit="${{2*pi}}"
    		wrist_2_lower_limit="${{-2*pi}}" wrist_2_upper_limit="${{2*pi}}"
    		wrist_3_lower_limit="${{-2*pi}}" wrist_3_upper_limit="${{2*pi}}"
  	/>

    <!-- Add a joint between the UR5's flange and the object -->
    <joint name="tool_center_point" type="fixed">
        <origin rpy="{} {} {}" xyz="{} {} {}"/>
        <parent link="ee_link"/>
        <child link="object"/>
    </joint>

    <!-- Include the composite object -->
    {}

</robot>
'''.format(object_rpy_wrt_ee[0], object_rpy_wrt_ee[1], object_rpy_wrt_ee[2], object_pos_wrt_ee[0], object_pos_wrt_ee[1], object_pos_wrt_ee[2], content)

#Write main XACRO file in current folder
main_xacro_basename = PBSM_PATH + "/models/UR5_compositeobject"
with open(main_xacro_basename+'.xacro', 'w') as fmain:
    fmain.write(main_xacro_content)

#Generate URDF from XACRO
command = 'xacro {} > {}'.format(main_xacro_basename+'.xacro', main_xacro_basename+'.urdf')
os.system(command)

#Load URDF and Simulate
cid = p.connect(p.GUI)#or p.DIRECT for non-graphical version

print('Current Simulation Parameters:')
print(p.getPhysicsEngineParameters(cid))

#The fixedTimeStep and numSolverIterations are the most important parameters to trade-off quality versus performance
fixedTimeStep = 1. / 240  #Default is 1./240
numSolverIterations = 500 #Default is 50

#Its best to keep default values for simulation parameters.
p.setPhysicsEngineParameter(numSolverIterations=numSolverIterations)
p.setTimeStep(fixedTimeStep)
p.setRealTimeSimulation(0)

#Only one additional search path can be supplied to PyBullet
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#Disable rendering during loading makes it much faster
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

#Instanciate the UR5+object
robot = UR5(cid, alternateModelFilename = main_xacro_basename+'.urdf')

#You might want to play with these values but use reasonable values.
robot.setMaxVelocity(0.25)
robot.setMaxForce(150)

#Initialize joints position
initial_joints_positions_degrees = [0, -90, 60, -90, -90, 0]
robot.initJoints(initial_joints_positions_degrees)

#Add a force-torque sensor on the ee_link_gripper_base_joint
gripper_base_index = robot.getJointIndexFromName('ee_fixed_joint')
p.enableJointForceTorqueSensor(robot.getUID(), gripper_base_index)

#Position the vizualizer camera
p.resetDebugVisualizerCamera(cameraDistance=0.5,cameraYaw=60,cameraPitch=0,cameraTargetPosition=[0.45,0,0.4])

#Re-enable rendering
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

#Set the downward gravity acceleration
p.setGravity(0,0,-9.81)

#Add visual reference frame on the Force-Torque Sensor
p.addUserDebugLine(lineFromXYZ=[0,0,0], lineToXYZ=[1,0,0], lineColorRGB=[1,0,0], lineWidth=0.1, lifeTime=0, parentObjectUniqueId=robot.getUID(), parentLinkIndex=robot.getJointIndexFromName('ee_fixed_joint'))
p.addUserDebugLine(lineFromXYZ=[0,0,0], lineToXYZ=[0,1,0], lineColorRGB=[0,1,0], lineWidth=0.1, lifeTime=0, parentObjectUniqueId=robot.getUID(), parentLinkIndex=robot.getJointIndexFromName('ee_fixed_joint'))
p.addUserDebugLine(lineFromXYZ=[0,0,0], lineToXYZ=[0,0,1], lineColorRGB=[0,0,1], lineWidth=0.1, lifeTime=0, parentObjectUniqueId=robot.getUID(), parentLinkIndex=robot.getJointIndexFromName('ee_fixed_joint'))

#Add visual reference frame on the End-effector. The positions are relative to the center of mass of the specified link
p.addUserDebugLine(lineFromXYZ=[0,0,0], lineToXYZ=[1,0,0], lineColorRGB=[1,0,0], lineWidth=0.1, lifeTime=0, parentObjectUniqueId=robot.getUID(), parentLinkIndex=robot.getJointIndexFromName('tool_center_point'))
p.addUserDebugLine(lineFromXYZ=[0,0,0], lineToXYZ=[0,1,0], lineColorRGB=[0,1,0], lineWidth=0.1, lifeTime=0, parentObjectUniqueId=robot.getUID(), parentLinkIndex=robot.getJointIndexFromName('tool_center_point'))
p.addUserDebugLine(lineFromXYZ=[0,0,0], lineToXYZ=[0,0,1], lineColorRGB=[0,0,1], lineWidth=0.1, lifeTime=0, parentObjectUniqueId=robot.getUID(), parentLinkIndex=robot.getJointIndexFromName('tool_center_point'))  

#Add the plane only after having initialized the position of the robot arm since moving the robot arm to its initial position can cause a collision
#between the plane and the robot arm.
planeId = p.loadURDF("plane.urdf")

#The hammer is added in the URDF
hammerId = 0

#Add visual reference frame on the Object. The positions are relative to the center of mass of the specified link
p.addUserDebugLine(lineFromXYZ=[0,0,0], lineToXYZ=[1,0,0], lineColorRGB=[1,0,0], lineWidth=0.1, lifeTime=0, parentObjectUniqueId=hammerId, parentLinkIndex=-1)
p.addUserDebugLine(lineFromXYZ=[0,0,0], lineToXYZ=[0,1,0], lineColorRGB=[0,1,0], lineWidth=0.1, lifeTime=0, parentObjectUniqueId=hammerId, parentLinkIndex=-1)
p.addUserDebugLine(lineFromXYZ=[0,0,0], lineToXYZ=[0,0,1], lineColorRGB=[0,0,1], lineWidth=0.1, lifeTime=0, parentObjectUniqueId=hammerId, parentLinkIndex=-1)


for i in range(0,8000):  
    p.stepSimulation()
    A = np.deg2rad(45)
    f0 = 0.7/10 * fixedTimeStep
    f1 = 1/10   * fixedTimeStep
    f2 = 1.3/10 * fixedTimeStep
    f3 = 1.6/10 * fixedTimeStep
    joint_goal = [  np.deg2rad(initial_joints_positions_degrees[0]),
                    np.deg2rad(initial_joints_positions_degrees[1]),
                    np.deg2rad(initial_joints_positions_degrees[2]),
                    np.deg2rad(initial_joints_positions_degrees[3])+A*np.sin(2*np.pi*f1*i), 
                    np.deg2rad(initial_joints_positions_degrees[4])+A*np.sin(2*np.pi*f2*i), 
                    np.deg2rad(initial_joints_positions_degrees[5])+A*np.sin(2*np.pi*f3*i)]
    robot.setGoal(joint_goal)
    time.sleep(fixedTimeStep)

#Disconnect from the physics server
p.disconnect()