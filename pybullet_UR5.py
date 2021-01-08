#!/usr/bin/python3
import sys
MIN_PYTHON = (3, 8)
if sys.version_info < MIN_PYTHON:
    sys.exit("Python %s.%s or later is required.\n" % MIN_PYTHON)

import os
import logging
import numpy as np
import pybullet as p

class UR5:
    def __init__(self, simulatorConnectionID, robotUID=None, startingPosition=[0,0,0.001], startingOrientationRAD=[0,0,0]):
        self._cid = simulatorConnectionID

        #If no robot unique id (robotUID) is supplied, we assume that we want to instantiate a new UR5
        if robotUID == None:
            startPos  = startingPosition
            startOrientation = p.getQuaternionFromEuler(startingOrientationRAD)
            
            dirname  = os.path.dirname(__file__)
            filename = os.path.join(dirname, 'models/UR5.urdf')
            self.arm = [p.loadURDF(filename,startPos, startOrientation)]
        else:
            self.arm = [robotUID]

        self.MAX_VELOCITY   = 0.25
        self.MAX_FORCE      = 150

    def setMaxVelocity(self, max_vel):
        #A sanity check should probably be done here
        self.MAX_VELOCITY = max_vel

    def setMaxForce(self, max_force):
        #A sanity check should probably be done here
        #Unreasonable forces can break everything.
        self.MAX_FORCE = max_force

    def getUID(self):
        return self.arm[0]

    def initJoints(self, jointsPositionDEG):
        p.resetJointState(targetValue=np.deg2rad(jointsPositionDEG[0]), bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('shoulder_pan_joint'))
        p.resetJointState(targetValue=np.deg2rad(jointsPositionDEG[1]), bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('shoulder_lift_joint'))
        p.resetJointState(targetValue=np.deg2rad(jointsPositionDEG[2]), bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('elbow_joint'))
        p.resetJointState(targetValue=np.deg2rad(jointsPositionDEG[3]), bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('wrist_1_joint'))
        p.resetJointState(targetValue=np.deg2rad(jointsPositionDEG[4]), bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('wrist_2_joint'))
        p.resetJointState(targetValue=np.deg2rad(jointsPositionDEG[5]), bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('wrist_3_joint'))

    #Returns the joints positions of the arm
    def getArmState(self):
        current_joint_position = [  p.getJointState(bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('shoulder_pan_joint'))[0],
                                    p.getJointState(bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('shoulder_lift_joint'))[0],
                                    p.getJointState(bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('elbow_joint'))[0],
                                    p.getJointState(bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('wrist_1_joint'))[0],
                                    p.getJointState(bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('wrist_2_joint'))[0],
                                    p.getJointState(bodyUniqueId=self.getUID(), jointIndex=self.getJointIndexFromName('wrist_3_joint'))[0] ]
        return current_joint_position

    #Returns joints position error from goal position
    def getStateError(self, arm_joint_positions_goal):
        state = self.getArmState()
        g = arm_joint_positions_goal
        error = np.linalg.norm([state[0]-g[0], state[1]-g[1], state[2]-g[2], state[3]-g[3], state[4]-g[4], state[5]-g[5]])
        return error

    #Uses position control to actuate each joint of the robot arm toward the goal
    #The joints_goal should be a list of 6 radians
    def setGoal(self, joints_goal):
        p.setJointMotorControl2(targetPosition=joints_goal[0], bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('shoulder_pan_joint'), controlMode=p.POSITION_CONTROL, maxVelocity=self.MAX_VELOCITY, force=self.MAX_FORCE)
        p.setJointMotorControl2(targetPosition=joints_goal[1], bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('shoulder_lift_joint'), controlMode=p.POSITION_CONTROL, maxVelocity=self.MAX_VELOCITY, force=self.MAX_FORCE)
        p.setJointMotorControl2(targetPosition=joints_goal[2], bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('elbow_joint'), controlMode=p.POSITION_CONTROL, maxVelocity=self.MAX_VELOCITY, force=self.MAX_FORCE)
        p.setJointMotorControl2(targetPosition=joints_goal[3], bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('wrist_1_joint'), controlMode=p.POSITION_CONTROL, maxVelocity=self.MAX_VELOCITY, force=self.MAX_FORCE)
        p.setJointMotorControl2(targetPosition=joints_goal[4], bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('wrist_2_joint'), controlMode=p.POSITION_CONTROL, maxVelocity=self.MAX_VELOCITY, force=self.MAX_FORCE)
        p.setJointMotorControl2(targetPosition=joints_goal[5], bodyIndex=self.getUID(), jointIndex=self.getJointIndexFromName('wrist_3_joint'), controlMode=p.POSITION_CONTROL, maxVelocity=self.MAX_VELOCITY, force=self.MAX_FORCE)


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