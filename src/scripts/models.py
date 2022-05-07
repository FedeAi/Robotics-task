"""
In this file there are all the models representing the robot,
those models are used by calibrator.py to perform parameter calibration
"""
import math
import numpy as np


class EncoderState:

    def __init__(self, CPR):
        self.previousStep = 0
        self.previousTime = 0
        self.CPR = CPR

    def computeAngularSpeed(self, timestamp, encoderValue):
        deltaEncoder = encoderValue - self.previousStep
        deltaTime = timestamp - self.previousTime
        self.previousTime = timestamp
        self.previousStep = encoderValue

        if(deltaTime > 0.2):
            return 0.0

        angularSpeed = deltaEncoder / deltaTime / self.CPR * 2 * math.pi

        return angularSpeed


class Kinematic:
    def __init__(self, r, l, gearRatio, wheelRadius):
        self.gearRatio = gearRatio
        self.wheelRadius = wheelRadius
        fact = 1.0 / (r + l)

        self.H_0p = np.array([[-fact, fact, fact, -fact],
                              [1, 1, 1, 1],
                              [-1, 1, -1, 1]])

    def computeKinematic(self, angularSpeed1, angularSpeed2, angularSpeed3, angularSpeed4):
        wheelSpeeds = np.array([angularSpeed1, angularSpeed2, angularSpeed3, angularSpeed4])
        V_b = np.dot(self.H_0p, wheelSpeeds.transpose()) * self.wheelRadius / self.gearRatio / 4.0
        return V_b  #[w,vx,vy]


class OdometryRK4:
    def __init__(self, startingPose):
        self.previousTime = 0.0
        self.pose = startingPose    # [theta, x, y]

    def updatePose(self, control, timestamp):
        deltaTime = timestamp - self.previousTime
        self.previousTime = timestamp

        self.pose = self.integrate__(control, deltaTime)

        return self.pose

    def integrate__(self, control : np.array, deltatime):     # control w ,vx, vy
        # RK4
        #K1
        k1_p = np.dot(self.computeRotation(self.pose[0]) ,control[1:3].transpose()) * deltatime
        deltaTheta = deltatime * control[0]

        #K2
        k2_p = np.dot(self.computeRotation(self.pose[0] + deltaTheta/2.0) ,control[1:3].transpose()) * deltatime

        #K3
        k3_p = np.dot(self.computeRotation(self.pose[0] + deltaTheta/2.0) ,control[1:3].transpose()) * deltatime

        #K4
        k4_p = np.dot(self.computeRotation(self.pose[0] + deltaTheta) ,control[1:3].transpose()) * deltatime

        newTheta = self.pose[0] + deltaTheta
        newPosition = self.pose[1:3] + k1_p / 6.0 + k2_p / 3.0 + k3_p / 3.0 + k4_p / 6.0
        return np.append(newTheta,newPosition)

    def computeRotation(self, theta):
        return np.array([[np.cos(theta), -np.sin(theta)],
                         [np.sin(theta), np.cos(theta)]])


class Robot:
    def __init__(self, w, l, r, gearRatio, CPR, startingPose):
        self.encoderFL = EncoderState(CPR=CPR)
        self.encoderFR = EncoderState(CPR=CPR)
        self.encoderRL = EncoderState(CPR=CPR)
        self.encoderRR = EncoderState(CPR=CPR)

        self.kinematic = Kinematic(w, l, gearRatio, r)
        self.odomCalc = OdometryRK4(startingPose)

    def updatePose(self, eFL, eFR, eRR, eRL, timestamp):
        FL_speed = self.encoderFL.computeAngularSpeed(timestamp, eFL)
        FR_speed = self.encoderFR.computeAngularSpeed(timestamp, eFR)
        RR_speed = self.encoderRR.computeAngularSpeed(timestamp, eRR)
        RL_speed = self.encoderRL.computeAngularSpeed(timestamp, eRL)

        V_b = self.kinematic.computeKinematic(FL_speed, FR_speed, RR_speed, RL_speed)

        return self.odomCalc.updatePose(V_b, timestamp)