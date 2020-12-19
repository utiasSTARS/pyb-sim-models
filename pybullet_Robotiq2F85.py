#!/usr/bin/python3
import sys
MIN_PYTHON = (3, 8)
if sys.version_info < MIN_PYTHON:
    sys.exit("Python %s.%s or later is required.\n" % MIN_PYTHON)

import logging
import numpy as np
import pybullet as p

#Note about the 2F-85:  
#The chassis and fingers are made out of anodized aluminum 6061 T6. Mass density = 0.0027 Kg/cm^3
#The finger bars are made out of stainless steel 17-4 ph. Mass density = 0.0028 Kg/cm^3
#The silicon of the finger tips is 60A

class Robotiq2F85:
    def __init__(self, simulatorConnectionID, robotUID=None, startingPosition=[0,0,0.001], startingOrientationRAD=[0,0,0]):

        self._cid = simulatorConnectionID

        #If no robot unique id (robotUID) is supplied, we assume that we want to instantiate a new 2F85
        if robotUID == None:
            startPos         = startingPosition
            startOrientation = p.getQuaternionFromEuler(startingOrientationRAD)
            self.gripper     = [p.loadURDF("./models/2F85.urdf",startPos, startOrientation)]
        else:
            self.gripper = [robotUID]

        #Maximum closing of the gripper
        self.MAX_CLOSING = 0.7
        #Maximum force of the gripper
        self.MAX_FORCE   = 10

        #We need to add a constraint for each finger of the gripper to close the kinematic loop.
        right_follower_index   = self.getJointIndexFromName('gripper_right_follower_joint')
        right_springlink_index = self.getJointIndexFromName('gripper_right_spring_link_joint')
        right_constraint = p.createConstraint(parentBodyUniqueId=self.getUID(), parentLinkIndex=right_follower_index, 
            childBodyUniqueId=self.getUID(), childLinkIndex=right_springlink_index, 
            jointType=p.JOINT_POINT2POINT, jointAxis=[1, 0, 0], parentFramePosition=[0, 0/1000, -15/1000], childFramePosition=[0, -2/1000, 32/1000])
        #Make the constraint very strong so an actuation cannot separate the links
        #For details about ERP: https://raw.githubusercontent.com/bulletphysics/bullet3/master/examples/Utils/b3ERPCFMHelper.hpp
        p.changeConstraint(right_constraint, maxForce=9999, erp=0.8)

        left_follower_index   = self.getJointIndexFromName('gripper_left_follower_joint')
        left_springlink_index = self.getJointIndexFromName('gripper_left_spring_link_joint')
        left_constraint = p.createConstraint(parentBodyUniqueId=self.getUID(), parentLinkIndex=left_follower_index, 
            childBodyUniqueId=self.getUID(), childLinkIndex=left_springlink_index, 
            jointType=p.JOINT_POINT2POINT, jointAxis=[1, 0, 0], parentFramePosition=[0, 0/1000, -15/1000], childFramePosition=[0, -2/1000, 32/1000])
        #Make the constraint very strong so an actuation cannot separate the links
        p.changeConstraint(left_constraint, maxForce=9999, erp=0.8)

        #There is a pin fixed on the driver link which limits the movement of the coupler link. If we dont use self-collision (for performances and stability purposes)
        #then we need to make sure that the coupler does not move through the pin. We can do that by limiting the rotation of the coupler/driver joint.
        p.changeDynamics(bodyUniqueId=self.getUID(), linkIndex=self.getJointIndexFromName('gripper_right_coupler_joint'), 
                        jointLowerLimit=np.deg2rad(-90) , jointUpperLimit=0)
        p.changeDynamics(bodyUniqueId=self.getUID(), linkIndex=self.getJointIndexFromName('gripper_left_coupler_joint'),  
                        jointLowerLimit=np.deg2rad(-90) , jointUpperLimit=0)

        #The spring links must not travel too far or they may collide with each other.
        p.changeDynamics(bodyUniqueId=self.getUID(), linkIndex=self.getJointIndexFromName('gripper_right_spring_link_joint'), 
                        jointLowerLimit=np.deg2rad(-45) , jointUpperLimit=0, jointDamping=0.5)
        p.changeDynamics(bodyUniqueId=self.getUID(), linkIndex=self.getJointIndexFromName('gripper_right_spring_link_joint'),  
                        jointLowerLimit=0 , jointUpperLimit=np.deg2rad(45), jointDamping=0.5)

        # WATCH OUT!!! In PyBullet, all revolute and slider joints have active motors by default. This does not work well for an underactuated mechanism.
        # Therefore, we need to disable some of them by setting a maximal force to 0 and allowing a full 360 degres rotation
        p.setJointMotorControl2(bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_right_follower_joint'), controlMode=p.VELOCITY_CONTROL, force=0)
        p.setJointMotorControl2(bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_left_follower_joint'),  controlMode=p.VELOCITY_CONTROL, force=0)
        p.changeDynamics(bodyUniqueId=self.getUID(), linkIndex=self.getJointIndexFromName('gripper_right_follower_joint'), jointLowerLimit=np.deg2rad(-360), jointUpperLimit=np.deg2rad(360))
        p.changeDynamics(bodyUniqueId=self.getUID(), linkIndex=self.getJointIndexFromName('gripper_left_follower_joint'),  jointLowerLimit=np.deg2rad(-360), jointUpperLimit=np.deg2rad(360))


        #The mass and inertia of the links are grossly inaccurate in most (all?) models available online.
        #But the impact of these values on the simulation can be very important.
        #The following values were obtained from the CAD supplied by Robotiq using the anodized aluminum 6061 T6 with mass density = 0.0027 Kg/cm^3
        #PyBullet/URDF uses Mass specified in Kg and inertia in Kg/m^2
        p.changeDynamics(bodyUniqueId=self.getUID(), linkIndex=self.getJointIndexFromName('gripper_right_spring_link_joint'), mass=29.95/1000, localInertiaDiagonal=[50.79/10 ,107.07/10 ,151.03/10 ])
        p.changeDynamics(bodyUniqueId=self.getUID(), linkIndex=self.getJointIndexFromName('gripper_right_driver_joint'),      mass=18.49/1000, localInertiaDiagonal=[6.64/10  ,34.75/10  ,37.50/10 ])
        p.changeDynamics(bodyUniqueId=self.getUID(), linkIndex=self.getJointIndexFromName('gripper_right_coupler_joint'),     mass=27.31/1000, localInertiaDiagonal=[22.27/10 ,69.13/10  ,85.40/10 ])
        p.changeDynamics(bodyUniqueId=self.getUID(), linkIndex=self.getJointIndexFromName('gripper_right_follower_joint'),    mass=19.55/1000, localInertiaDiagonal=[9.98/10  ,42.82/10  ,45.92/10 ])
        p.changeDynamics(bodyUniqueId=self.getUID(), linkIndex=self.getJointIndexFromName('gripper_right_pad_joint'),         mass=15.55/1000, localInertiaDiagonal=[6.01/10  ,17.63/10  ,21.98/10 ])
        p.changeDynamics(bodyUniqueId=self.getUID(), linkIndex=self.getJointIndexFromName('gripper_left_spring_link_joint'),  mass=29.95/1000, localInertiaDiagonal=[50.79/10 ,107.07/10 ,151.03/10 ])
        p.changeDynamics(bodyUniqueId=self.getUID(), linkIndex=self.getJointIndexFromName('gripper_left_driver_joint'),       mass=18.49/1000, localInertiaDiagonal=[6.64/10  ,34.75/10  ,37.50/10 ])
        p.changeDynamics(bodyUniqueId=self.getUID(), linkIndex=self.getJointIndexFromName('gripper_left_coupler_joint'),      mass=27.31/1000, localInertiaDiagonal=[22.27/10 ,69.13/10  ,85.40/10 ])
        p.changeDynamics(bodyUniqueId=self.getUID(), linkIndex=self.getJointIndexFromName('gripper_left_follower_joint'),     mass=19.55/1000, localInertiaDiagonal=[9.98/10  ,42.82/10  ,45.92/10 ])
        p.changeDynamics(bodyUniqueId=self.getUID(), linkIndex=self.getJointIndexFromName('gripper_left_pad_joint'),          mass=15.55/1000, localInertiaDiagonal=[6.01/10  ,17.63/10  ,21.98/10 ])

        #Initialize the state of the gripper to fully open
        p.resetJointState(bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_right_driver_joint'), targetValue=0)
        p.resetJointState(bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_left_driver_joint'),  targetValue=0)

    def setMaxForce(self, max_force):
        self.MAX_FORCE = max_force
   
    def getUID(self):
        return self.gripper[0]

    def initJoints(self, jointsPositionDEG):
        #Initialize the state of the gripper to fully open
        p.resetJointState(bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_right_driver_joint'), targetValue=jointsPositionDEG[0])
        p.resetJointState(bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_left_driver_joint'),  targetValue=jointsPositionDEG[1])

    #Returns the joints positions of the gripper
    def getGripperState(self):
        current_joint_position = [  p.getJointState(bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_left_driver_joint'))[0],
                                    p.getJointState(bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_right_driver_joint'))[0] ]
        return current_joint_position

    #Returns joints position error from goal position
    def getStateError(self, gripper_joint_positions_goal):
        g = gripper_joint_positions_goal
        state = self.getGripperState()
        error = np.linalg.norm([state[0]-g[0]*(self.MAX_CLOSING/100),state[1]-g[1]*(self.MAX_CLOSING/100)])
        return error

    def getMaximumClosing(self):
        #The driver joint is motorized. The value of 0.7 for maximum closing has been found by trial and error
        return self.MAX_CLOSING

    #Uses position control to actuate each driver joint of the gripper
    #Each of left_finger_goal, right_finger_goal is a percentage that specify how close each finger is (100 = Fully closed, 0 = Fully open)
    def setGoal(self, left_finger_goal, right_finger_goal):
        #All the values here takes a lot of time to tune...modify them at your own risk.
        
        right_finger_goal = max(min(right_finger_goal,100),0)
        left_finger_goal  = max(min(left_finger_goal,100),0)

        p.setJointMotorControl2(targetPosition=right_finger_goal*self.MAX_CLOSING/100, bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_right_driver_joint'), controlMode=p.POSITION_CONTROL, maxVelocity=1)
        p.setJointMotorControl2(targetPosition=left_finger_goal*self.MAX_CLOSING/100,  bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_left_driver_joint'),  controlMode=p.POSITION_CONTROL, maxVelocity=1)
        
        #Acts as the spring in the gripper that keeps the pads parallel
        p.setJointMotorControl2(targetPosition=self.MAX_CLOSING, bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_right_spring_link_joint'), controlMode=p.POSITION_CONTROL, force=10)
        p.setJointMotorControl2(targetPosition=self.MAX_CLOSING, bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('gripper_left_spring_link_joint'),  controlMode=p.POSITION_CONTROL, force=10) 

    #Build a dictionnary that can then be used to efficiently retrieve
    #joints unique IDs from their names.
    def buildJointIndexFromNameDict(self):
        self.jointsNamesDict = {}

        numJoints = p.getNumJoints(self.getUID(), self._cid)

        for i in range(numJoints):
            jointInfo = p.getJointInfo(self.getUID(), i, self._cid)
            strJointName = jointInfo[1].decode('UTF-8')
            self.jointsNamesDict[strJointName] = jointInfo[0]

    #Retrieve the unique ID associated with a joint name.
    #Returns None if the joint name is not found
    def getJointIndexFromName(self, JointName):
        #Verify that the joints dictonary has been populated
        #if its not the case, do it.
        try:
            if len(self.jointsNamesDict) == 0:
                self.buildJointIndexFromNameDict()
        except AttributeError:
            self.buildJointIndexFromNameDict()

        jointId = self.jointsNamesDict.get(JointName)
        if jointId == None:
            logging.warning('Cannot find a joint named: '+JointName)
        return jointId