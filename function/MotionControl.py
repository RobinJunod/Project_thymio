# Project Mobile Robotics - Group 31 - Motion Control

import numpy as np
import math
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle

class MotionControl:
    def __init__(self,Kp = 50,Ki= 0.1,Kd = 0.1):
        self.e_t_previous = [0,0,0]
        self.e_t_actual = [0,0,0]         #sera égal à xgoal-xest , ygoal-yest, thetagoal-thetaest
        self.de_t = [0,0,0]
        self.ie_t_actual = [0,0,0]
        self.ie_t_previous = [0,0,0]
        self.u_t = [0,0,0]
        self.Kp = Kp  #a voir si besoin de différents Kp ....
        self.Ki = Ki
        self.Kd = Kd
        self.timeStep = 1/240 #240Hz = frequency for the discretization
        self.rayon = 20 #mm rayon d'une roue
        self.L = 95 #mm distance entre les roues
        self.omega_r = 0 
        self.omega_l = 0 
        self.vx = 0 
        self.omega = 0

        self.pos_est = [0,0,0]
        
        
        


    def PID(self):
        #on calcul les integrales et derivees
        self.ie_t_actual = self.ie_t_previous+self.e_t_previous*self.timeStep

        self.de_t = (self.e_t_actual-self.e_t_previous)/self.timeStep
        
        self.u_t = self.Kp*self.e_t_actual + self.Ki*self.ie_t + self.Kd*self.de_t

        self.e_t_previous = self.e_t_actual
        self.ie_t_previous = self.ie_t_actual


    def Plant(self):
        self.vx = (self.e_t_actual[0]-self.e_t_previous[0])/self.timeStep
        self.omega = (self.e_t_actual[2]-self.e_t_previous[2])/self.timeStep

        self.omega_r = self.vx/self.rayon-self.L/2*self.omega/self.rayon
        self.omega_l = self.vx/self.rayon+self.L/2*self.omega/self.rayon


    def IntegrationVitesse(self):
        v = (self.omega_r+self.omega_l)/2
        R = self.L/2*(self.omega_r+self.omega_l)/(self.omega_r-self.omega_l)
        self.pos_est[2] = 2*np.pi*R*self.timeStep
        self.pos_est[0] = 0
        self.pos_est[1] = 0


