#%%
################################## IMPORTS #########################################
# basic computation library
import cv2
import numpy as np
import math

# package for threading
import time
import threading
from threading import Timer

# Created files
#import Thymio
import MotionControl
import vision
import filtering
import Global_Navigation
import LocalNavigation
import visualisation

# package for thymio
from tdmclient import ClientAsync, aw


########################### SET GLOBAL VARIABLES ################################
# Golbal variables (varaibles that must be shared btwn threads)
global PROX_SENSOR, ODOMETRY

# Global constants
THYMIO_RADIUS = 47 # in [mm]
THYMIO_SPEED_CONVERTION = 0.3175373
MM_TO_PIX_CONVERTION = 438/1070

#############
CAMERA = 2

# &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
"""
CREATE THE THYMIO CLASS THAT WILL BE USED IN THREADS, IN ORDER NOT TO HAVE A 
GLOBAL VARIABLE NODE
"""
class thymio():
    def __init__(self) -> None:
        self.client = ClientAsync()
        self.node = aw(self.client.wait_for_node())
        aw(self.node.lock()) 
        
    def set_speed(self, left_speed, right_speed) -> None:
        self.node.send_set_variables(set_speed(left_speed, right_speed))
    
    def get_speed(self) -> tuple:
        return [self.node["motor.left.speed"], self.node["motor.right.speed"]]
    
    def get_sensor(self) -> list:
        return list(self.node["prox.horizontal"])
# CREATE THE THYMIO object

thymio = thymio()
locNav = LocalNavigation.LocalNavigation()
# &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
"""         _                           _         
           | |                         | |        
   ___   __| | ___  _ __ ___   ___ _ __| |_ _   _ 
  / _ \ / _` |/ _ \| '_ ` _ \ / _ \ '__| __| | | |
 | (_) | (_| | (_) | | | | | |  __/ |  | |_| |_| |
  \___/ \__,_|\___/|_| |_| |_|\___|_|   \__|\__, |
                                             __/ |
                                            |___/ 
"""
lock_ODOMETRY = threading.Lock()
#ODOMETRY = [0,0,0, time.time(), 0]
def thread_update_odometry():
    """                                    
    This thread is executed in continus and is there to update the global variable ODOMETRY
    Args:
        thymio (): will use the global variables node and ODOMETRY
    """
    # take glabal variable
    global ODOMETRY, THYMIO_RADIUS, THYMIO_SPEED_CONVERTION, MM_TO_PIX_CONVERTION
    while 1:
        speed_l = thymio.get_speed()[0]
        speed_r = thymio.get_speed()[1]
        # convert speed in mm/s
        speed_l = speed_l * THYMIO_SPEED_CONVERTION * MM_TO_PIX_CONVERTION
        speed_r = speed_r * THYMIO_SPEED_CONVERTION * MM_TO_PIX_CONVERTION
        
        [pre_pos_x, pre_pos_y, pre_angle, previous_time, d_time] = ODOMETRY
        # delta time btwn last and new speed recording
        d_time = time.time() - previous_time
        # compute new angle
        d_angle = (speed_r - speed_l)/(4*THYMIO_RADIUS^2) * d_time
        angle = pre_angle + d_angle
        # compute new pos
        direction_x = math.cos((pre_angle + angle)/2)
        direction_y = math.sin((pre_angle + angle)/2)

        pos_x = pre_pos_x + (speed_l + speed_r)/2 * direction_x * d_time
        pos_y = pre_pos_y + (speed_l + speed_r)/2 * direction_y * d_time
        previous_time = time.time()
        
        # output update ODOMETRY
        lock_ODOMETRY.acquire()
        ODOMETRY = [pos_x, pos_y, angle, previous_time, d_time]
        lock_ODOMETRY.release()
        time.sleep(0.1)
        
Thread_odometry = threading.Thread(target=thread_update_odometry,)


# &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&                          
"""
  ___  ___ _ __  ___  ___  _ __ 
 / __|/ _ \ '_ \/ __|/ _ \| '__|
 \__ \  __/ | | \__ \ (_) | |   
 |___/\___|_| |_|___/\___/|_|   
"""                     
                                
lock_PROX_SENSOR = threading.Lock()
PROX_SENSOR = [0, 0, 0, 0, 0, 0, 0]
def thread_get_sensor():
    global PROX_SENSOR
    while 1:
        prox_sens_values = thymio.get_sensor()
        
        # update PROXIMITY_SENSOR
        lock_PROX_SENSOR.acquire()
        PROX_SENSOR = prox_sens_values
        lock_PROX_SENSOR.release()
        
        time.sleep(0.1)

Thread_sensor = threading.Thread(target=thread_get_sensor,)

# &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&


def set_speed(left, right):
    
    #small function to be used with node.send_set_variables()
    
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }   
"""
def get_data():
    #Test function TOO: remove it
    
    global ODOMETRY
    while 1:
        ODOMETRY = [node["motor.left.speed"], node["motor.right.speed"]]
"""
# &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

"""
                  _         _                   
                 (_)       | |                  
  _ __ ___   __ _ _ _ __   | | ___   ___  _ __  
 | '_ ` _ \ / _` | | '_ \  | |/ _ \ / _ \| '_ \ 
 | | | | | | (_| | | | | | | | (_) | (_) | |_) |
 |_| |_| |_|\__,_|_|_| |_| |_|\___/ \___/| .__/ 
                                         | |    
                                         |_|    
"""

# Main Thread
if __name__ == '__main__':
    # Open camera
    cap = cv2.VideoCapture(CAMERA)
    # Check if camera opened successfully
    if (cap.isOpened()== False): 
        print("Error opening video stream or file")
    else:
        print("Camera is openend")

    # Need this loop to work
    while(cap.isOpened()):
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
        
        ##############################################################################################

        ##### VISUALIZATION #########################################################################
        visualisation.visualisation(rescMap, POS_VISION[0], POS_VISION[1], obst_coords, ANGLE_VISION, shortestpath, goal_coords[0], goal_coords[1])
        cv2.waitKey(0)
        ##### END VISUALIZATION #####################################################################
        
        # Init threads and Thymio
        aw(thymio.node.wait_for_variables()) # wait for Thymio variables values

        # Init odometry
        ODOMETRY = [POS_VISION[0], POS_VISION[1], ANGLE_VISION, time.time(), 0]
        # Start Threads
        Thread_odometry.start()
        Thread_sensor.start()
        #init pid controller and variable for motion control and filtering
        #thymio_angle = ODOMETRY[2]
        #thymio_pos = [ODOMETRY[0], ODOMETRY[1]]
        Sigma_angle = 0 # Init sigma for filtering
        Sigma_pos = 0 # Init sigma for fiiltering
        ind_next =1 # Init index for next goal on shortestpath
        goal_pos = shortestpath[ind_next]
        PID = MotionControl.MotionControl()

        print("Let's go")
        PID.update_angle_error(ODOMETRY[2], [ODOMETRY[0], ODOMETRY[1]], goal_pos)
        d_time = 0
        # Calculate speed from angle error and set initial speed
        [left_speed, right_speed] = PID.PID(d_time, 100, 100)
        thymio.set_speed(math.floor(left_speed), math.floor(right_speed))
        time_last_ctrl = time.time()
        #&&&&&&&&&&&&&&&&&&&&&&& main loop (while thymio not in goal region) &&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        while 1:
            # time.sleep would not work here, use asynchronous client.sleep method instead
            aw(thymio.client.sleep(0.1))

            ########### Update Thymio Position and Angle ##################################################
            #lock_ODOMETRY.acquire()
            #thymio_angle = ODOMETRY[2]
            #thymio_pos = [ODOMETRY[0], ODOMETRY[1]]
            #lock_ODOMETRY.release()
            ### Via vision
            # Capture new frame
            ret, frame = cap.read()
            # Return Thymio coordinates and angle from vision
            rescMap, POS_VISION, ANGLE_VISION = vision.updateThymioPos(frame, M, MAPwidth, MAPheight)
            if np.isnan(POS_VISION[0]) or np.isnan(POS_VISION[1]):
                POS_VISION = [ODOMETRY[0], ODOMETRY[1]]
                print("Position lost")
            if np.isnan(ANGLE_VISION):
                ANGLE_VISION = ODOMETRY[2]
            ##############################################################################################
            ################## FILTERING #################################################################
            ODOMETRY[2], Sigma_angle = filtering.kalmanFilterAngle(ODOMETRY[2], ANGLE_VISION, Sigma_angle)
            ODOMETRY[0], ODOMETRY[1], Sigma_pos = filtering.kalmanFilterPos(ODOMETRY[0], ODOMETRY[1], POS_VISION[0], POS_VISION[1], Sigma_pos)

            # check if goal reached
            if (abs(ODOMETRY[0]-goal_pos[0]) < 15 and abs(ODOMETRY[1]-goal_pos[1]) < 15):
                if ind_next == len(shortestpath)-1:
                    thymio.set_speed(0, 0)
                    print("Destination reached ! Thank you for flying with us")
                    break
                else:
                    ind_next += 1
            # update goal point
            goal_pos = shortestpath[ind_next]

            # PID controller
            PID.update_angle_error(ODOMETRY[2], [ODOMETRY[0], ODOMETRY[1]], goal_pos)
            d_time = time.time() - time_last_ctrl
            [left_speed, right_speed] = PID.PID(d_time, 100, 100)
            thymio.set_speed(math.floor(left_speed), math.floor(right_speed))
            time_last_ctrl = time.time()
            ###############################################################################################
            #lock_PROX_SENSOR.acquire()
            #print(PROX_SENSOR)
            #lock_PROX_SENSOR.release()
            # Stop obstacle condtion
            if(locNav.detect_obstacle(PROX_SENSOR)) :
                left_speed, right_speed = locNav.turn_if_obstacle(PROX_SENSOR)
                thymio.set_speed(math.floor(left_speed), math.floor(right_speed))
                time.sleep(0.3)
                left_speed, right_speed = locNav.go_straight()
                thymio.set_speed(math.floor(left_speed), math.floor(right_speed))
                time.sleep(1.2)
            ##### VISUALIZATION #########
            visualisation.visuDuringRun(rescMap, ODOMETRY[0], ODOMETRY[1], obst_coords, ODOMETRY[2], ANGLE_VISION, shortestpath, goal_coords[0], goal_coords[1])
            ##### END VISUALIZATION ############
                

            if cv2.waitKey(1) == 27:
                break
        break

    cv2.destroyAllWindows()
    # When everything done, release the video capture object
    cap.release()
    aw(thymio.node.unlock())
# %%
