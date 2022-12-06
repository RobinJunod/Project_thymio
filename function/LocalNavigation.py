import numpy as np
import math
import time


class LocalNavigation:
    def _init_(self, proximity_sensors):
        self.prox_horizontal = proximity_sensors[0:4] 

    def detect_obstacle(self, proximity_sensors, threshold=100):  
        self.prox_horizontal = proximity_sensors[0:4] 
        maxvalue = max(self.prox_horizontal)
        if(maxvalue > threshold) : 
            detect = True
        else :
            detect = False
        return detect


    def turn_if_obstacle(self, proximity_sensors): 
        self.prox_horizontal = proximity_sensors[0:4] 
        w_l = [20,  30, -30, -30, -20]
        w_r = [-20, -30, 30,  30,  20]

        # Scale factors for sensors 
        sensor_scale = 400
    
        y = [0,0]
        x = [0,0,0,0,0]
        

      
        for i in range(0,len(x)-1):
            # Get and scale inputs
            x[i] = self.prox_horizontal[i] // sensor_scale
            
            # Compute outputs of neurons and set motor powers
            y[0] = y[0] + x[i] * w_l[i]
            y[1] = y[1] + x[i] * w_r[i]    

        # Set motor powers
        motor_left_target = y[0]
        motor_right_target = y[1]

        #self.set_speed([motor_left_target,motor_right_target])

        return [motor_left_target,motor_right_target]

    def go_straight(self, speeds) :
        NOMINAL_SPEED = 160
        DELTA_SPEED = 60
        if(speeds[0]<speeds[1]) :
            motor_left_target = NOMINAL_SPEED
            motor_right_target = NOMINAL_SPEED - DELTA_SPEED
        else :
            motor_left_target = NOMINAL_SPEED - DELTA_SPEED
            motor_right_target = NOMINAL_SPEED


        return [motor_left_target,motor_right_target]
