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
global PROX_SENSOR, ODOMETRY, OBSTACLE

# Global constants
THYMIO_RADIUS = 47 # in [mm]
THYMIO_SPEED_CONVERTION = 0.3175373
LIST_ODO = []
# Camera number and bool for thread 
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
def thread_update_odometry(event):
    """                                    
    This thread is executed in continus and is there to update the global variable ODOMETRY
    Args:
        thymio (): will use the global variables node and ODOMETRY
    """
    # take glabal variable
    global ODOMETRY, THYMIO_RADIUS, THYMIO_SPEED_CONVERTION, LIST_ODO
    while 1:
        speed_l = thymio.get_speed()[0]
        speed_r = thymio.get_speed()[1]
        # convert speed in mm/s
        speed_l = speed_l * THYMIO_SPEED_CONVERTION * vision.pix_per_mm
        speed_r = speed_r * THYMIO_SPEED_CONVERTION * vision.pix_per_mm
        
        [pre_pos_x, pre_pos_y, pre_angle, previous_time, d_time] = ODOMETRY
        # delta time btwn last and new speed recording
        d_time = time.time() - previous_time
        # compute new angle
        d_angle = ((speed_r - speed_l)/(2*THYMIO_RADIUS* vision.pix_per_mm)) * d_time
        angle = pre_angle + d_angle
    
        if angle >= np.pi:
            angle = angle - 2*np.pi 
        elif angle <= -np.pi:
            angle = 2*np.pi + angle
        else:
            angle = angle
           
        # compute new pos
        direction_x = math.cos((pre_angle + angle)/2)
        direction_y = math.sin((pre_angle + angle)/2)

        pos_x = pre_pos_x + (speed_l + speed_r)/2 * direction_x * d_time
        pos_y = pre_pos_y - (speed_l + speed_r)/2 * direction_y * d_time
        previous_time = time.time()
        
        # output update ODOMETRY
        lock_ODOMETRY.acquire()
        ODOMETRY = [pos_x, pos_y, angle, previous_time, d_time]
        LIST_ODO.append(ODOMETRY)
        lock_ODOMETRY.release()
        if event.is_set():
            break
        time.sleep(0.1)
        



# &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&                          
"""
  ___  ___ _ __  ___  ___  _ __ 
 / __|/ _ \ '_ \/ __|/ _ \| '__|
 \__ \  __/ | | \__ \ (_) | |   
 |___/\___|_| |_|___/\___/|_|   
"""                     
                                
lock_PROX_SENSOR = threading.Lock()
lock_OBSTACLE = threading.Lock()
PROX_SENSOR = [0, 0, 0, 0, 0, 0, 0]
OBSTACLE = False
def thread_get_sensor(event):
    global PROX_SENSOR, OBSTACLE
    while 1:
        prox_sens_values = thymio.get_sensor()
        
        # update PROXIMITY_SENSOR
        lock_PROX_SENSOR.acquire()
        PROX_SENSOR = prox_sens_values
        lock_PROX_SENSOR.release()

        
        if(locNav.detect_obstacle(PROX_SENSOR)) :           
            bool_obstacle = True
        else :
            bool_obstacle = False

        lock_OBSTACLE.acquire()
        OBSTACLE = bool_obstacle
        lock_OBSTACLE.release()
        if event.is_set():
            break
        
        time.sleep(0.1)
    print("Thread closing")
 

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
def main():
    global ODOMETRY, PROX_SENSOR, MM_TO_PIX_CONVERTION
    RUNNING = threading.Event()
    Thread_odometry = threading.Thread(target=thread_update_odometry, args=(RUNNING,))
    Thread_sensor = threading.Thread(target=thread_get_sensor, args=(RUNNING,))
    # Open camera
    cap = cv2.VideoCapture(CAMERA)
    # Check if camera opened successfully
    if (cap.isOpened()== False): 
        print("Error opening video stream or file")
    else:
        print("Camera is opened")

    # While video stream opened
    while(cap.isOpened()):
    ########## Take first picture for init and initialize MAP ###################################
        for i in range(1,20):
            ret, frame = cap.read()

        # Deal with problem at reading
        while ret == False:
            print("Can't receive frame. Retrying ...")
            cap.release()
            cap = cv2.VideoCapture(CAMERA)
            for i in range(1,20):
                ret, frame = cap.read()

        # Initialize MAP
        rescMap, M, IMGwidth, IMGheight, mapSize, goal_coords, obst_coords, POS_VISION, ANGLE_VISION = vision.map_init(frame)

        ##############################################################################################
        ############ Path planning ###################################################################
        globNav = Global_Navigation.Global_Navigation(obst_coords,POS_VISION,goal_coords,mapSize)        
        shortestpath=globNav.create_path()
            
        ##############################################################################################
        ##### VISUALIZATION #########################################################################
        visualisation.visualisation(rescMap, POS_VISION[0], POS_VISION[1], obst_coords, ANGLE_VISION, shortestpath, goal_coords[0], goal_coords[1])
        cv2.waitKey(0) # Start the robot
        ##### END VISUALIZATION #####################################################################
            
        # Init threads and Thymio 
        aw(thymio.node.wait_for_variables()) # wait for Thymio variables values

        # Init odometry
        ODOMETRY = [POS_VISION[0], POS_VISION[1], ANGLE_VISION, time.time(), 0]
        # Start Threads
        Thread_odometry.start()
        Thread_sensor.start()

        #init pid controller and variable for motion control and filtering
        Sigma_angle = 0.01 # Init sigma for filtering
        Sigma_pos = 10 # Init sigma for filtering
        ind_next = 1 # Init index for next goal on shortestpath
        goal_pos = shortestpath[ind_next]
        d_time = 0
        PID = MotionControl.MotionControl()

        print("Let's go")
        PID.update_angle_error(ODOMETRY[2], [ODOMETRY[0], ODOMETRY[1]], goal_pos)
        # Calculate speed from angle error and set initial speed
        [left_speed, right_speed] = PID.PID(d_time, 100, 100)
        thymio.set_speed(math.floor(left_speed), math.floor(right_speed))
        time_last_ctrl = time.time()
        #&&&&&&&&&&&&&&&&&&&&&&& main loop (while thymio not in goal region) &&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        while 1:
            if(OBSTACLE == False) :
                # time.sleep would not work here, use asynchronous client.sleep method instead
                aw(thymio.client.sleep(0.1))

                ########### Update Thymio Position and Angle ##################################################
                # Capture new frame
                ret, frame = cap.read()
                # Return Thymio coordinates and angle from vision
                rescMap, POS_VISION, ANGLE_VISION = vision.updateThymioPos(frame, M, IMGwidth, IMGheight)
               
                # Do filtering if not nan
                lock_ODOMETRY.acquire()
                if ~np.isnan(POS_VISION[0]) and ~np.isnan(POS_VISION[1]) and ~np.isnan(ANGLE_VISION):
                    ODOMETRY[2], Sigma_angle = filtering.kalmanFilterAngle(ODOMETRY[2], ANGLE_VISION, Sigma_angle)
                    ODOMETRY[0], ODOMETRY[1], Sigma_pos = filtering.kalmanFilterPos(ODOMETRY[0], ODOMETRY[1], POS_VISION[0], POS_VISION[1], Sigma_pos)
                    LIST_ODO.append(ODOMETRY)
                else:
                    print("Position lost")
                    Sigma_angle = Sigma_angle + 0.008
                    Sigma_pos = Sigma_pos + 0.5
                lock_ODOMETRY.release()
                ##############################################################################################
                # check if goal reached
                if ((ODOMETRY[0]-goal_pos[0])**2 + (ODOMETRY[1]-goal_pos[1])**2 < 225):
                    if ind_next == len(shortestpath)-1:
                        thymio.set_speed(0, 0)
                        print("Destination reached ! Thank you for flying with us")
                        cap.release()
                        break
                    else:
                        ind_next += 1
                        PID.PID_integral = 0
                elif ((ODOMETRY[0]-goal_pos[0])**2 + (ODOMETRY[1]-goal_pos[1])**2 > 4500 + (shortestpath[ind_next-1][0]-goal_pos[0])**2 + (shortestpath[ind_next-1][1]-goal_pos[1])**2):
                    ############ Path planning ###################################################################
                    globNav = Global_Navigation.Global_Navigation(obst_coords,[ODOMETRY[0],ODOMETRY[1]],goal_coords,mapSize)        
                    shortestpath=globNav.create_path()
                    ind_next = 1
                    PID.PID_integral = 0
                # update goal point
                
                goal_pos = shortestpath[ind_next]

                # PID controller
                PID.update_angle_error(ODOMETRY[2], [ODOMETRY[0], ODOMETRY[1]], goal_pos)
                d_time = time.time() - time_last_ctrl
                [left_speed, right_speed] = PID.PID(d_time, 100, 100)
                thymio.set_speed(math.floor(left_speed), math.floor(right_speed))
                time_last_ctrl = time.time()

                ##### VISUALIZATION #########
                visualisation.visuDuringRun(rescMap, ODOMETRY[0], ODOMETRY[1], Sigma_pos, obst_coords, ODOMETRY[2], Sigma_angle, shortestpath, goal_coords[0], goal_coords[1])
                ##### END VISUALIZATION ############
            else :
                ##### LOCAL NAVIGATION ######### 
                while(OBSTACLE) :
                    left_speed, right_speed = locNav.turn_if_obstacle(PROX_SENSOR)
                    thymio.set_speed(math.floor(left_speed), math.floor(right_speed))
                    aw(thymio.client.sleep(0.1))    
                left_speed, right_speed = locNav.curvilinear_traj([left_speed,right_speed])
                thymio.set_speed(math.floor(left_speed), math.floor(right_speed))   
                aw(thymio.client.sleep(2.5))     

            if cv2.waitKey(1) == 27:
                break
        break
        
    thymio.set_speed(0, 0)
    RUNNING.set()
    Thread_odometry.join()
    Thread_sensor.join()
    cap.release()
  
    cv2.destroyAllWindows()
    aw(thymio.node.unlock())

if __name__ == '__main__':
    main()
