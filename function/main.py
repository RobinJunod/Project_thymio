#%%
################################## IMPORTS #########################################
# basic computation library
import cv2
import numpy as np
import math
import random # for test onla
import sys
# package for thymio
from tdmclient import ClientAsync
#from tdmclient.atranspiler import ATranspiler

# package for threading
import time
from threading import Timer
import asyncio


# Created files
#import Thymio
import MotionControl
import vision
import setThymioSpeed
import filtering
import Global_Navigation
import visualisation

########################### SET GLOBAL VARIABLES ################################
# Golbal variables (varaibles that must be shared btwn threads)
global PROXIMITY_SENSOR, ODOMETRY, speedl, speedr

# Global constants
THYMIO_RADIUS = 47 # in [mm]
THYMIO_SPEED_CONVERTION = 0.3175373

####### Testing without camera ???? ####
WITHOUT_CAMERA = False
I_PIC = 1
#############
CAMERA = 2
#k_sobel = 5

MAP_WIDTH = 1070 # mm
THYMIO_WIDTH = 110 # mm

RED_L = np.array([105,75,129],np.uint8)  # Set the Lower and higher range value of color for filter
RED_H = np.array([179,160,255],np.uint8)

GREEN_L = np.array([0, 60, 90],np.uint8)
GREEN_H = np.array([87, 255, 255],np.uint8)

BLUE_L = np.array([0, 79, 154],np.uint8)
BLUE_H = np.array([156, 255, 255],np.uint8)

YELLOW_L = np.array([0, 0, 158],np.uint8)
YELLOW_H = np.array([179, 240, 255],np.uint8)

D1 = 100.0 # Initialize distance to goal to something bigger than the threshold
D2 = 100.0

client = ClientAsync()


async def getSpeed():
    node = await client.wait_for_node()
    await node.lock()
    await node.wait_for_variables({"motor.left.speed"})
    await node.wait_for_variables({"motor.right.speed"})
    speedl = node.v.motor.left.speed
    speedr = node.v.motor.right.speed
    await node.unlock()
    update_odometry(speedl,speedr)
       
def runSpeed():
    client.run_async_program(getSpeed)       
        

###### for filtering tests #####
def update_odo():
    ODOMETRY[2] = ODOMETRY[2] + random.randrange(-20,20)/100
####################################

# Threading function 
def update_odometry(vL,vR):
    """Odometry part, thread that gives us the thymio position and angle based on the
    motor speed measured. Change global variable odometry.

    Args:
        thymio (thymio object): the thymio object
    """
    # take glabal variable
    global ODOMETRY, THYMIO_RADIUS, THYMIO_SPEED_CONVERTION

    # convert speed in mm/s
    speed_l = vL * THYMIO_SPEED_CONVERTION
    speed_r = vR * THYMIO_SPEED_CONVERTION
    
    [pre_pos_x, pre_pos_y, pre_angle, previous_time, d_time] = ODOMETRY
    # delta time btwn last and new speed recording
    d_time = time.time() - previous_time
    # compute new angle
    d_angle = (speed_r - (speed_l-2))/(4*THYMIO_RADIUS^2) * d_time
    angle = pre_angle + d_angle
    print(angle)
    # compute new pos
    direction_x = math.cos((pre_angle + angle)/2)
    direction_y = math.sin((pre_angle + angle)/2)

    pos_x = pre_pos_x + (speed_l + speed_r)/2 * direction_x * d_time
    pos_y = pre_pos_y - (speed_l + speed_r)/2 * direction_y * d_time
    previous_time = time.time()
    # output update ODOMETRY
    ODOMETRY = [pos_x, pos_y, angle, previous_time, d_time]
    

################## MAIN #################################################

def main():
    global ODOMETRY, I_PIC, POS_VISION, ANGLE_VISION, D1, D2 
    
    # Open camera
    cap = cv2.VideoCapture(CAMERA)
    # Check if camera opened successfully
    if (cap.isOpened()== False): 
        print("Error opening video stream or file")
    else:
        print("Camera is openend")


    # Need this loop to work
    while(cap.isOpened()) or (WITHOUT_CAMERA == True):
        ########## Take first picture for init and initialize MAP ###################################
        for i in range(1,20):
            ret, frame = cap.read()
            i +=1
        # Deal with problem at reading
        while ret == False:
            print("Can't receive frame. Retrying ...")
            cap.release()
            cap = cv2.VideoCapture(CAMERA)
            for i in range(1,20):
                ret, frame = cap.read()
                i+=1
        # Initialize MAP
        rescMap, M, MAPwidth, MAPheight, b_coords, goal_coords, obst_coords, POS_VISION, ANGLE_VISION = vision.map_init(frame)
        ##############################################################################################

        ############ Path planning ###################################################################
        mapSize = [MAPwidth, MAPheight]
        globNav = Global_Navigation.Global_Navigation(obst_coords,POS_VISION,goal_coords,mapSize)        
        shortestpath=globNav.create_path()
        print(shortestpath)
        ##############################################################################################

        ##### VISUALIZATION #########################################################################
        visualisation.visualisation(rescMap, POS_VISION[0], POS_VISION[1], obst_coords, ANGLE_VISION, shortestpath, goal_coords[0], goal_coords[1])
        cv2.waitKey(0)
        ##### END VISUALIZATION #####################################################################

        time.sleep(1)
        ODOMETRY = [POS_VISION[0], POS_VISION[1], ANGLE_VISION, time.time(), 0.1]
        Sigma_angle = 0
        ind_next =1
        next_point = shortestpath[ind_next]
        previous_time = time.time()
        PID = MotionControl.MotionControl()

        print("Activate Thymio")
        print("Let's go")

        # Entering main loop (while thymio not in goal region)
        while D1 > 15 or D2 > 15:
            ########### Motion control ####################################################################
            # check if goal reached
            if (abs(ODOMETRY[0]-next_point[0]) < 15 and abs(ODOMETRY[1]-next_point[1]) < 15):
                ind_next += 1
            # update goal point
            next_point = shortestpath[ind_next]
            # update angle error
            PID.update_angle_error(ODOMETRY[2], [ODOMETRY[0], ODOMETRY[1]], next_point)
            # compute PID speed
            [robot_speed_l, robot_speed_r] = PID.PID(ODOMETRY[4], 102, 100)
            vL = math.floor(robot_speed_l)
            vR = math.floor(robot_speed_r)
            setSpeed = setThymioSpeed.setThymioSpeed()
            setSpeed.runClas(vL, vR)
            ODOMETRY[3] = time.time()
            ###############################################################################################

            ########### Update Thymio Position and Angle ##################################################
            # Capture new frame
            ret, frame = cap.read()
            # Return Thymio coordinates and angle from Odometry
            runSpeed()
            # Return Thymio coordinates and angle from vision
            rescMap, POS_VISION, ANGLE_VISION = vision.updateThymioPos(frame)
            print("Vision angle = " + str(ANGLE_VISION))
            ##############################################################################################


            ################## FILTERING ###############################
            newANGLE, Sigma_angle = filtering.kalmanFilterAngle(ODOMETRY[2], ANGLE_VISION, Sigma_angle)


            ##### VISUALIZATION #########
            visualisation.visuDuringRun(rescMap, POS_VISION[0], POS_VISION[1], obst_coords, newANGLE, ANGLE_VISION, shortestpath, goal_coords[0], goal_coords[1])
            ##### END VISUALIZATION ############
            ODOMETRY[2] = newANGLE
            # Compute distance to goal
            D1 = abs(POS_VISION[0]-goal_coords[0])
            D2 = abs(POS_VISION[1]-goal_coords[1])
            
            ########## Filtering should go here ############
            #ODOMETRY[2], Sigma_angle = filtering.kalmanFilterAngle(ODOMETRY[2],ANGLE_VISION, Sigma_angle)
            #print("Vision angle = " + str(ANGLE_VISION))
            #print("Filtered = " + str(ODOMETRY[2]))
            #update_odo()
            #print("After odometry = " + str(ODOMETRY[2]))
            ##################################################
            # Press esc on keyboard to  exit
            if cv2.waitKey(1) == 27:
                break
        break
    setSpeed.runClas(0, 0)
    cv2.destroyAllWindows()
    # When everything done, release the video capture object
    cap.release()

if __name__ == '__main__':
    # Run only when this is the main file
    main()