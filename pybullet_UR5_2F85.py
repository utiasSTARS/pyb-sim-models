#!/usr/bin/python3
import sys
MIN_PYTHON = (3, 8)
if sys.version_info < MIN_PYTHON:
    sys.exit("Python %s.%s or later is required.\n" % MIN_PYTHON)

import os
import numpy as np
import pybullet as p
from pybullet_UR5 import UR5
from pybullet_Robotiq2F85 import Robotiq2F85

class UR5_2F85:
    def __init__(self, simulatorConnectionID, startingPosition=[0,0,0.001], startingOrientationRAD=[0,0,0]):
        self._cid = simulatorConnectionID
        startPos  = startingPosition
        startOrientation = p.getQuaternionFromEuler(startingOrientationRAD)

        dirname  = os.path.dirname(__file__)
        filename = os.path.join(dirname, 'models/UR5_2F85.urdf')
        self.robot = [p.loadURDF(filename,startPos, startOrientation)]

        self.ur5     = UR5(self._cid, robotUID=self.getUID())
        self.Rob2f85 = Robotiq2F85(self._cid, robotUID=self.getUID())

    def setGripperMaxForce(self, max_force):
        self.Rob2f85.setMaxForce(max_force)

    def setArmMaxVelocity(self, max_vel):
        self.ur5.setMaxVelocity(max_vel)

    def setArmMaxForce(self, max_force):
        self.ur5.setMaxForce(max_force)

    def getUID(self):
        return self.robot[0]

    def initJoints(self, jointsPositionDEG):
        ur5_joints  = jointsPositionDEG[0:6]
        r2f85       = jointsPositionDEG[6:8]
        self.ur5.initJoints(ur5_joints)
        self.Rob2f85.initJoints(r2f85)
        
    #Computes the error between the goal state and the current state
    #The state is [position_end_effector, orientation_end_effector, left_gripper_closing, right_gripper_closing]
    def stateError(self, arm_joint_positions_goal, gripper_joint_positions_goal):
        arm_error       = self.ur5.getStateError(arm_joint_positions_goal)
        gripper_error   = self.Rob2f85.getStateError(gripper_joint_positions_goal)
        state_error     = np.linalg.norm([arm_error, gripper_error])
        return  state_error

    #Given the current joints positions and a relative joints motion provided in relative_joints
    #computes the goal in terms of the absolute positions of the joints
    def relativeJointsToAbsolute(self, relative_joints):
        arm_rel_joints      = relative_joints[0]
        gripper_rel_joints  = relative_joints[1]

        arm_current_joints_pos      = self.ur5.getArmState()
        gripper_current_joints_pos  = self.Rob2f85.getGripperState()

        absolute_joints = [arm_current_joints_pos[0]+arm_rel_joints[0],
                            arm_current_joints_pos[1]+arm_rel_joints[1],
                            arm_current_joints_pos[2]+arm_rel_joints[2],
                            arm_current_joints_pos[3]+arm_rel_joints[3],
                            arm_current_joints_pos[4]+arm_rel_joints[4],
                            arm_current_joints_pos[5]+arm_rel_joints[5],
                            gripper_current_joints_pos[0]*100/self.Rob2f85.MAX_CLOSING+gripper_rel_joints[0],
                            gripper_current_joints_pos[1]*100/self.Rob2f85.MAX_CLOSING+gripper_rel_joints[1]]
        return absolute_joints

    #Given current goal and a list of next goals to iteratively reach, verify if the current goal has been reached
    #Call the appropriate functions to move toward the next goal
    def setCurrentGoal(self):
        #If the error between current state and goal is below this threshold, consider that the goal is reached
        ERROR_THRESHOLD = 0.0075

        #The first time setCurrentGoal() is called, self.arm_joint_positions_goal and self.gripper_joint_positions_goal wont exists
        #and we need to initialize them (see below except AttributeError)
        try:
            #If the error is below the treshold AND we have not yet reached the end of the sequence
            state_error  = self.stateError(self.arm_joint_positions_goal, self.gripper_joint_positions_goal)
            if state_error < ERROR_THRESHOLD and (self.robotGoalsSequenceIndex + 1) < len(self.robotGoalsSequence):
                #Increment the sequence index
                self.robotGoalsSequenceIndex = self.robotGoalsSequenceIndex + 1
                #Compute the joints positions goals from the cartesian goal
                current_goal                  = self.robotGoalsSequence[self.robotGoalsSequenceIndex]
                #If the goal is expressed as 4 terms, then we need to compute the inverse kinematics
                #If the goal is expressed as 2 terms, then it means that the goal is expressing relative joint positions
                #If the goal is expressed as 8 terms, then it means that the goal is expressing absolute joint positions
                if len(current_goal) == 4:
                    self.arm_joint_positions_goal = p.calculateInverseKinematics(bodyUniqueId=self.getUID(), endEffectorLinkIndex=self.ur5.getJointIndexFromName('tool_center_point'), 
                                                                                targetPosition=current_goal[0], targetOrientation=current_goal[1],
                                                                                residualThreshold=0.001, maxNumIterations=100)
                    self.gripper_joint_positions_goal   = current_goal[2:4]
                if len(current_goal) == 2:
                    abs_goal = self.relativeJointsToAbsolute(current_goal)
                    self.arm_joint_positions_goal       = abs_goal[0:6]
                    self.gripper_joint_positions_goal   = abs_goal[6:8]
                if len(current_goal) == 8:
                    self.arm_joint_positions_goal       = current_goal[0:6]
                    self.gripper_joint_positions_goal   = current_goal[6:8]

            #This is run as often as possible to keep the position control accurate
            self.setArmJointsGoal(self.arm_joint_positions_goal)
            self.setGripperGoal(self.gripper_joint_positions_goal[0], self.gripper_joint_positions_goal[1])
        except AttributeError:
            #Compute the joints positions goals from the cartesian goal
            current_goal                  = self.robotGoalsSequence[self.robotGoalsSequenceIndex]
            self.arm_joint_positions_goal = p.calculateInverseKinematics(bodyUniqueId=self.getUID(), endEffectorLinkIndex=self.ur5.getJointIndexFromName('tool_center_point'), 
                                                                        targetPosition=current_goal[0], targetOrientation=current_goal[1],
                                                                        residualThreshold=0.001, maxNumIterations=100)
            self.gripper_joint_positions_goal = [current_goal[2], current_goal[3]]
            self.setArmJointsGoal(self.arm_joint_positions_goal)
            self.setGripperGoal(self.gripper_joint_positions_goal[0], self.gripper_joint_positions_goal[1])

        next_goal = self.robotGoalsSequence[self.robotGoalsSequenceIndex]

        return next_goal

    #Add a goal, which is defined in term of the robot state, to the list the robot will go through sequencially 
    #Robot state is: [position_end_effector, orientation_end_effector, left_gripper_closing, right_gripper_closing]
    def addGoal(self, goal):
        #Verify that the list of goals exists, if it does not, then initialize it.
        try:
            self.robotGoalsSequence.append(goal)
        except AttributeError:
            self.initGoalSequence()
            self.robotGoalsSequence.append(goal)

    def initGoalSequence(self):
        self.robotGoalsSequence = []
        self.robotGoalsSequenceIndex = 0

    #Uses position control to actuate each joint of the robot arm toward the goal
    #The joints_goal should be a list of 6 radians
    def setArmJointsGoal(self, joints_goal):
        self.ur5.setGoal(joints_goal)

    #Uses position control to actuate each driver joint of the gripper
    #Each of left_finger_goal, right_finger_goal is a percentage that specify how close each finger is (100 = Fully closed, 0 = Fully open)
    def setGripperGoal(self, left_finger_goal, right_finger_goal):
        self.Rob2f85.setGoal(left_finger_goal, right_finger_goal)

    def printJointsInfo(self):
        numJoints = p.getNumJoints(self.getUID(), self._cid)
        for i in range(numJoints):
            jointInfo = p.getJointInfo(self.getUID(), i, self._cid)
            print('Index: \t\t'+str(jointInfo[0]))
            print('Name: \t\t'+str(jointInfo[1]))
            print('Type: \t\t' + str(jointInfo[2]))
            print('qIndex: \t' + str(jointInfo[3]))
            print('uIndex: \t' + str(jointInfo[4]))
            print('Flags: \t\t' + str(jointInfo[5]))
            print('Damping: \t' + str(jointInfo[6]))
            print('Friction: \t' + str(jointInfo[7]))
            print('LowerLimit: \t' + str(jointInfo[8]))
            print('UpperLimit: \t' + str(jointInfo[9]))
            print('MaxForce: \t' + str(jointInfo[10]))
            print('MaxVelocity: \t' + str(jointInfo[11]))
            print('LinkName: \t' + str(jointInfo[12]))
            print('Axis: \t\t' + str(jointInfo[13]))
            print('FramePos: \t' + str(jointInfo[14]))
            print('FrameOrient: \t' + str(jointInfo[15]))
            print('ParentIndex: \t' + str(jointInfo[16]))
            print('----')

    def printLinksInfo(self):
        numJoints = p.getNumJoints(self.getUID(), self._cid)
        for i in range(numJoints):
            linkInfo  = p.getDynamicsInfo(bodyUniqueId=self.getUID(), linkIndex=i)
            jointInfo = p.getJointInfo(self.getUID(), i)
            print('Name: \t' + str(jointInfo[12]))
            print('Mass: \t\t'+str(linkInfo[0]))
            print('Lateral friction coeff.: \t\t'+str(linkInfo[1]))
            print('Local inertia diagonal: \t\t' + str(linkInfo[2]))
            print('Position of inertial frame: \t' + str(linkInfo[3]))
            print('Orient. of inertial frame: \t' + str(linkInfo[4]))
            print('Coeff. of restitution: \t\t' + str(linkInfo[5]))
            print('Rolling friction coeff.: \t' + str(linkInfo[6]))
            print('Spinning friction coeff.: \t' + str(linkInfo[7]))
            print('Contact damping: \t' + str(linkInfo[8]))
            print('Contact stiffness: \t' + str(linkInfo[9]))
            print('Body type: \t' + str(linkInfo[10]))
            print('Collision margin: \t' + str(linkInfo[11]))
            print('----')