#!/usr/bin/env python

"""
Abridged controller code for project 1
Author: Chris Correa, Valmik Prabhu
"""

import numpy as np

class Controller:

    def __init__(self, sim=None):
        """
        Constructor for the superclass. All subclasses should call the superconstructor

        Parameters
        ----------
        sim : SimpleArmSim object. Contains all information about simulator state (current joint angles and velocities, robot dynamics, etc.)
        """

        if sim:
            self.sim = sim

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        makes a call to the robot to move according to it's current position and the desired position 
        according to the input path and the current time. Each Controller below extends this 
        class, and implements this accordingly.  

        Parameters
        ----------
        target_position : (2,) numpy array 
            desired position or joint angles
        target_velocity : (2,) numpy array
            desired end-effector velocity or joint velocities
        target_acceleration : (2,) numpy array 
            desired end-effector acceleration or joint accelerations

        Returns
        ----------
        desired control input (joint velocities or torques) : (2,) numpy array
        """
        pass


class JointVelocityController(Controller):

    def __init__(self, sim=None):
        super().__init__(sim)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        Sets the joint velocity equal to that computed through IK

        Parameters
        ----------
        target_position : (2,) numpy array 
            desired positions or joint angles
        target_velocity : (2,) numpy array
            desired end-effector velocity or joint velocities
        target_acceleration : (2,) numpy array 
            desired end-effector acceleration or joint accelerations

        ----------
        desired control input (joint velocities or torques) : (2,) numpy array
        """
        return target_velocity

class WorkspaceVelocityController(Controller):

    def __init__(self, sim=None):
        super().__init__(sim)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        Computes joint velocities to execute given end effector body velocity

        Parameters
        ----------
        target_position : (2,) numpy array 
            desired positions or joint angles
        target_velocity : (2,) numpy array
            desired end-effector velocity or joint velocities
        target_acceleration : (2,) numpy array 
            desired end-effector acceleration or joint accelerations

        Returns
        ----------
        desired control input (joint velocities or torques) : (2,) numpy array
        """
        pass


class JointTorqueController(Controller):

    def __init__(self, sim=None):
        super().__init__(sim)

    def step_control(self, target_position, target_velocity, target_acceleration):
        """
        Recall that in order to implement a torque based controller you will need access to the 
        dynamics matrices M, C, G such that

        M ddq + C dq + G = u

        Look in section 4.5 of MLS for theory behind the computed torque control law.

        Parameters
        ----------
        target_position : (2,) numpy array 
            desired positions or joint angles
        target_velocity : (2,) numpy array
            desired end-effector velocity or joint velocities
        target_acceleration : (2,) numpy array 
            desired end-effector acceleration or joint accelerations

        Returns
        ----------
        desired control input (joint velocities or torques) : (2,) numpy array
        """
        pass
