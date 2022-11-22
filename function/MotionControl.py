# Project Mobile Robotics - Group 31 - Motion Control

import numpy as np
import math
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle

class MotionControl:
    def __init__(self,Kp,Ki,Kd):
        #self.CoMHeight = pelvisHeight # We assume that CoM and pelvis are the same point
        #self.stepDuration = stepTiming
        #self.timeStep = 1/240 #We select this value for the timestep(dt) for discretization of the trajectory. The 240 Hz is the default numerical solving frequency of the pybullet. Therefore we select this value for DCM trajectory generation discretization.
        #self.numberOfSamplesPerSecond =  240 #Number of sampling of the trajectory in each second
        #self.numberOfSteps = 14 #This is the desired number of steps for walking
        #self.DCM = list("")
        #self.gravityAcceleration=9.81
        #self.omega = math.sqrt(self.gravityAcceleration/self.CoMHeight ) #Omega is a constant value and is called natural frequency of linear inverted pendulum
        #mobile
        self.e_t_previous = 0
        self.e_t_actual = 0          #sera égal à xgoal-xest , ygoal-yest, thetagoal-thetaest
        self.de_t = 0
        self.ie_t_actual = 0
        self.ie_t_previous = 0
        self.u_t = 0
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.timeStep = 1/240 #240Hz = frequency for the discretization
        
        pass


    def PID(self):
        #on calcul les integrales et derivees
        self.ie_t_actual = self.ie_t_previous+self.e_t_previous*self.timeStep

        self.de_t = (self.e_t_actual-self.e_t_previous)/self.timeStep
        
        self.u_t = self.Kp*self.e_t_actual + self.Ki*self.ie_t + self.Kd*self.de_t

        self.e_t_previous = self.e_t_actual
        self.ie_t_previous = self.ie_t_actual




    def getDCMTrajectory(self):
        self.findFinalDCMPositionsForEachStep() #or we can have another name for this function based on equation (8) of the jupyter notebook: for example findInitialDCMPositionOfEachStep()
        self.planDCM() #Plan DCM trajectory 
        return np.array(self.DCM)


    def getCoMTrajectory(self,com_ini):
        #This class generates the CoM trajectory by integration of CoM velocity(that has been found by the DCM values)
        self.CoM = np.zeros_like(self.DCM)
        self.CoMDot = np.zeros_like(self.DCM)
        self.CoM[0] = com_ini
        self.CoMDot[0] = 0

        # columns = X, Y, Z

        #todo: Use equation (3) in jupyter notebook to update "self.CoMDot" array
        for i in range(1,len(self.DCM)):
            self.CoMDot[i] = self.omega *(self.DCM[i]-self.CoM[i])
            self.CoMDot[i,2] = 0   
            self.CoM[i] = self.CoM[i-1]+self.CoMDot[i-1]*self.timeStep
            self.CoM[i,2] = self.CoMHeight
        #todo: Use numerical integration(for example a simple euler method) for filling the "self.CoM" array
        #Note: that "self.CoM" should be a 3d vector that third component is constant CoM height


    def setCoP(self, CoP):
        self.CoP = CoP #setting CoP positions. Note: The CoP has an offset with footprint positions 
        pass

    def setFootPrints(self,footPrints):
        self.footPrints = footPrints #setting footprint positions. Note: The footprint has an offset with CoP positions 


    def findFinalDCMPositionsForEachStep(self):# Finding Final(=initial for previous, refer to equation 8) dcm for a step
        self.DCMForEndOfStep = np.copy(self.CoP) #initialization for having same shape
        #todo: implement capturability constraint(3rd item of jupyter notebook steps for DCM motion planning section)
        #todo: Use equation 7 for finding DCM at the end of step and update the "self.DCMForEndOfStep" array  
        
        self.DCMForEndOfStep[-2] = self.CoP[-1]

        for i in range(self.CoP.shape[0]-2, 0, -1) : 
            time =  i*self.timeStep
            self.DCMForEndOfStep[i-1] = self.CoP[i]+(self.DCMForEndOfStep[i]-self.CoP[i])*np.exp(self.omega*(time))
        pass

    def calculateCoPTrajectory(self):
        self.DCMVelocity = np.zeros_like(self.DCM)
        self.CoPTrajectory = np.zeros_like(self.DCM)
        self.DCMVelocity[0] = 0
        self.CoPTrajectory[0] = self.CoP[0]
        #todo: Implement numerical differentiation for finding DCM Velocity and update the "self.DCMVelocity" array
        #todo: Use equation (4) to find CoP by having DCM and DCM Velocity and update the "self.CoPTrajectory" array
        
        for i in range(1,len(self.DCM)-1):
            self.DCMVelocity[i] = (self.DCM[i+1]-self.DCM[i])/self.timeStep
            self.CoPTrajectory[i] = self.DCM[i]-self.DCMVelocity[i]/self.omega

        pass


    def planDCM(self): #The output of this function is a DCM vector with a size of (int(self.numberOfSamplesPerSecond* self.stepDuration * self.CoP.shape[0])) that is number of sample points for whole time of walking
        for iter in range(int(self.numberOfSamplesPerSecond* self.stepDuration * self.CoP.shape[0])):# We iterate on the whole simulation control cycles:  
            time =  iter*self.timeStep #Finding the time of a corresponding control cycle
            i = int(time/self.stepDuration)  #Finding the number of corresponding step of walking
            t = time%self.stepDuration  #The “internal” step time t is reset at the beginning of each step
            
            self.DCM.append(self.CoP[i]+(self.DCMForEndOfStep[i]-self.CoP[i])*np.exp(self.omega*(t-time))) #Use equation (9) for finding the DCM trajectory
        pass

    
