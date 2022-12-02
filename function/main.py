#%%
################################## IMPORTS #########################################
# basic computation library
import cv2
import numpy as np
import math
import random # for test onla
# package for thymio
from tdmclient import ClientAsync, aw
from tdmclient.atranspiler import ATranspiler

# package for threading
import time
import asyncio


# Created files
#import Thymio
import MotionControl
import vision
import setThymioSpeed
#import filtering
import Global_Navigation

########################### SET GLOBAL VARIABLES ################################
# Golbal variables (varaibles that must be shared btwn threads)
global PROXIMITY_SENSOR, ODOMETRY

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

# VARIABLE INIT
rescMapTOT = []
t_coordsTOT = []
t_angleTOT = []



# init ODOMERTRY
ODOMETRY = [0, 0, 0, time.time()]

################################# THREADS #######################################

#%%

###### for filtering tests #####
def update_odo():
    ODOMETRY[2] = ODOMETRY[2] + random.randrange(-20,20)/100
####################################

# Threading function 
def update_odometry(thymio):
    """Odometry part, thread that gives us the thymio position and angle based on the
    motor speed measured. Change global variable odometry.

    Args:
        thymio (thymio object): the thymio object
   
    # take glabal variable
    global ODOMETRY, THYMIO_RADIUS, THYMIO_SPEED_CONVERTION

    #threading.Timer(0.5, update_odometry, [thymio]).start()
    #[speed_l, speed_r] = thymio.get_speed()
    # convert speed in mm/s
    speed_l = speed_l * THYMIO_SPEED_CONVERTION
    speed_r = speed_r * THYMIO_SPEED_CONVERTION
    
    [pre_pos_x, pre_pos_y, pre_angle, previous_time] = ODOMETRY
    # delta time btwn last and new speed recording
    d_time = time.time() - previous_time
    # compute new angle
    d_angle = (speed_l - speed_r)/(4*THYMIO_RADIUS^2) * d_time
    angle = pre_angle + d_angle
    # compute new pos
    direction_x = math.cos((pre_angle + angle)/2)
    direction_y = math.sin((pre_angle + angle)/2)

    pos_x = pre_pos_x + (speed_l + speed_r)/2 * direction_x * d_time
    pos_y = pre_pos_y + (speed_l + speed_r)/2 * direction_y * d_time
    previous_time = time.time()
    # output update ODOMETRY
    ODOMETRY = pos_x, pos_y, angle, previous_time, d_time
    """

################## MAIN #################################################

def main():
    global ODOMETRY, I_PIC, POS_VISION, ANGLE_VISION, D1, D2 
    
    #starting_pos = [0,0]
    #starting_angle = [0]
    #starting_goal = [1000,1000]
  
    # Open camera
    cap = cv2.VideoCapture(CAMERA)
    # Check if camera opened successfully
    if (cap.isOpened()== False): 
        print("Error opening video stream or file")
    else:
        print("Camera is openend")

    #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Need this loop to work
    while(cap.isOpened()) or (WITHOUT_CAMERA == True):
        # Take first picture for init
        ####### In case of testing without camera #######
        if WITHOUT_CAMERA == True:
            ret = True
            rescMap = cv2.imread('Images/ThymioTest0.jpg', cv2.IMREAD_COLOR)
            ### Find Goal
            goal_coords = vision.findGoal(rescMap,RED_L,RED_H,20,255)

            ### Find Obstacles
            obst_coords, imOBST = vision.findObst(rescMap,BLUE_L,BLUE_H,20,10)

            ### Find Thymio
            POS_VISION, ANGLE_VISION = vision.findThymio(rescMap, GREEN_L, GREEN_H,3,17,20)
        else:
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
        previous_time = time.time()
        ############ Path planning #############
        mapSize = [MAPwidth, MAPheight]
        test = Global_Navigation.Global_Navigation(obst_coords,POS_VISION,goal_coords,mapSize)        
        shortestpath=test.create_path()
        print(shortestpath)
        #test.plot_visibility_graph()

        ################################
        
        
        ##### VISUALIZATION #########
        x_g = math.floor(goal_coords[0])
        y_g = math.floor(goal_coords[1])
        x_t = math.floor(POS_VISION[0])
        y_t = math.floor(POS_VISION[1])
        imageInit = cv2.circle(rescMap, (x_g,y_g), 30, (0,0,255), 2)
        for obst in obst_coords:
            pts = obst
            pts = pts.reshape((-1,1,2))
            imageInit = cv2.polylines(imageInit,[pts.astype(np.int32)],True,(255,0,0))
        imageInit = cv2.line(imageInit,(x_t,y_t),(x_t+30,math.floor(y_t-30*np.tan(ANGLE_VISION))),(0,0,0),5)
        imageInit = cv2.circle(imageInit, (x_t,y_t), 30, (0,255,0), 2)
        imageInit = cv2.polylines(imageInit, np.int32([np.array(shortestpath).reshape((-1, 1, 2))]), False, (200, 0, 255), 3)
        cv2.imshow("Display window", imageInit)
        cv2.waitKey(0)
        ##### END VISUALIZATION ############

        time.sleep(1)
        ODOMETRY[0] = POS_VISION[0]
        ODOMETRY[1] = POS_VISION[1]
        ODOMETRY[2] = ANGLE_VISION
        Sigma_angle = 0
        ind_next =1
        next_point = shortestpath[ind_next]
        print(goal_coords)
        print(POS_VISION)
        print("Activate Thymio")
        print("Let's go")
        #thymio1 = Thymio.thymio(POS_VISION, ANGLE_VISION)

        # Entering main loop (while thymio not in goal region)
        while D1 > 60 or D2 > 60:
            ########### Motion control should go here ###########
            #time.sleep(1)
            robot_angle = ODOMETRY[2]
            robot_pos = [ODOMETRY[0], ODOMETRY[1]]

            if (abs(ODOMETRY[0]-next_point[0]) < 20 and abs(ODOMETRY[1]-next_point[1]) < 20):
                ind_next += 1

            next_point = shortestpath[ind_next]
            PID = MotionControl.MotionControl()
            # update angle
            PID.update_angle_error(robot_angle, robot_pos, next_point)
            # compute PID speed
            d_time = time.time() - previous_time
            #d_time = ODOMETRY[4]
            [robot_speed_l, robot_speed_r] = PID.PID(d_time, 100, 100)
            vL = math.floor(robot_speed_l)
            vR = math.floor(robot_speed_r)
            tutu = setThymioSpeed.setThymioSpeed()
            tutu.runClas(vL, vR)
            


            
            ########### Local Avoidance should go here (in background) ##########
            
            
            ########### Update Thymio Position and Angle ############
            # Capture new frame
            if WITHOUT_CAMERA == True:
                ret = True
                img_name = "Images/ThymioTest" + str(I_PIC) +".jpg"
                I_PIC += 1
                rescMap = cv2.imread(img_name, cv2.IMREAD_COLOR)
                ### Find Thymio
                POS_VISION, ANGLE_VISION = vision.findThymio(rescMap, GREEN_L, GREEN_H,3,17,20)
            else:
                ret, frame = cap.read()
                # Return Thymio coordinates and angle
                rescMap, POS_VISION, ANGLE_VISION = vision.updateThymioPos(frame)

            ODOMETRY[0] = POS_VISION[0]
            ODOMETRY[1] = POS_VISION[1]
            ODOMETRY[2] = ANGLE_VISION

            ##### VISUALIZATION #########
            image = cv2.circle(rescMap, (math.floor(POS_VISION[0]),math.floor(POS_VISION[1])), 30, (0,255,0), 2)
            x_t = math.floor(POS_VISION[0])
            y_t = math.floor(POS_VISION[1])
            image = cv2.circle(image, (x_g,y_g), 30, (0,0,255), 2)
            for obst in obst_coords:
                pts = obst
                pts = pts.reshape((-1,1,2))
                image = cv2.polylines(image,[pts.astype(np.int32)],True,(255,0,0))
            image = cv2.line(image,(x_t,y_t),(x_t+30,math.floor(y_t-30*np.tan(ANGLE_VISION))),(0,0,0),5)
            image = cv2.circle(image, (x_t,y_t), 30, (0,255,0), 2)
            image = cv2.polylines(image, np.int32([np.array(shortestpath).reshape((-1, 1, 2))]), False, (200, 0, 255), 3)
            cv2.imshow("Display window", image)
            ##### END VISUALIZATION ############

            # Compute distance to goal
            D1 = abs(POS_VISION[0]-goal_coords[0])
            D2 = abs(POS_VISION[1]-goal_coords[1])
            

            previous_time = time.time()
            ########## Filtering should go here ############
            #ODOMETRY[2], Sigma_angle = filtering.kalmanFilterAngle(ODOMETRY[2],ANGLE_VISION, Sigma_angle)
            #print("Vision angle = " + str(ANGLE_VISION))
            #print("Filtered = " + str(ODOMETRY[2]))
            #update_odo()
            #print("After odometry = " + str(ODOMETRY[2]))
            ##################################################
            time.sleep(0.1)
            # Press esc on keyboard to  exit
            if cv2.waitKey(1) == 27:
                break
        break
    tutu.runClas(0, 0)
    cv2.destroyAllWindows()

    
        
    if WITHOUT_CAMERA == False:
        # When everything done, release the video capture object
        cap.release()

    # thymio init
    #thymio1 = Thymio.thymio(starting_pos, starting_angle)

    
    # call thread function
    #check_prox_sensor(thymio1)
    #update_odometry(thymio1)
    # create PID object
    #PID = MotionControl()
    # initatate PID
    #PID.update_angle_error(starting_angle, starting_pos, starting_goal)
  
  
    """
    while 1:
        finished = False
        time.sleep(1)
        robot_angle = ODOMETRY[2]
        robot_pos = [ODOMETRY[0], ODOMETRY[1]]
        # update angle
        PID.update_angle_error(robot_angle, robot_pos, starting_angle)
        # compute PID speed
        d_time = ODOMETRY[4]
        [robot_speed_l, robot_speed_r] = PID.PID(d_time, 100, 100)
        thymio1.set_speed([robot_speed_l,robot_speed_r])
        
        print(ODOMETRY)
        if finished:
            thymio1.set_speed([0,0])
            break
    """

if __name__ == '__main__':
    # Run only when this is the main file
    main()


#%%


"""
async def print_sensor_values(sensor_id, print_range=10, delta_time=0.2):
    
    Print the sensor value sensor_id print_range times, every delta_time seconds
    
    await node.wait_for_variables({str(sensor_id)})
    for i in range(print_range):
        print(list(node[sensor_id]))
        await client.sleep(delta_time)  
await print_sensor_values('prox.ground.reflected')

def local_avoid(speed_robot, node):
    # global prox_and_memory
    prox_and_memory[0:5] = np.array([x for x in node['prox.horizontal'][0:5]])
    w= np.array([[80, 20, -25, -20, -80, 12, 0],[-80, -20, -20, 20, 80, 0, 12]])
    v= w.dot(prox_and_memory)
    if (abs(v) > 2200).any():
        prox_and_memory[5] = 0
        prox_and_memory[6] = 0
        speed_robot = (v / k_ann).astype(int) + offset_ann
    else:
        speed_robot += (v / 200).astype(int)

    prox_and_memory[5:7] = (speed_robot / 10).astype(int)

    return speed_robot


from threading import Timer
class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False
"""

