#%%
################################## IMPORTS #########################################
# basic computation library
import numpy as np
import math
# package for thymio
from tdmclient import ClientAsync, aw
from tdmclient.atranspiler import ATranspiler

# package for threading
import time
import asyncio
import threading

# Created files
import Thymio
import MotionControl

########################### SET GLOBAL VARIABLES ################################
# Golbal variables (varaibles that must be shared btwn threads)
global PROXIMITY_SENSOR, ODOMETRY
global THYMIO_DATA

# Global constants
THYMIO_RADIUS = 47 # in [mm]
THYMIO_SPEED_CONVERTION = 0.3175373


# init ODOMERTRY
ODOMETRY = [0, 0, 0, time.time()]

################################# THREADS #######################################

class thymio:
    def __init__(self, pos, angle) -> None:
        self.pos = pos
        self.angle = angle
        self.e_dist = np.array([0,0])
        # timer
        self.time = time.time()
        
        self.client = ClientAsync()
        self.client.process_waiting_messages()
        self.node = aw(self.client.wait_for_node())
        aw(self.node.lock())
    
    async def set_speed(self, speed_robot):
        #def motors(speed_robot):
        #    return {
        #        "motor.left.target": [speed_robot[0]],
        #        "motor.right.target": [speed_robot[1]],
        #    }
        #self.node.send_set_variables(motors(speed_robot))
        #await self.node.lock()
        v = {
            "motor.left.target":  [speed_robot[0]],
            "motor.right.target": [speed_robot[1]],
        }
        await self.node.set_variables(v)
        #await self.node.unlock()
        
        
    async def get_speed(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        
        #aw(self.node.lock())
        aw(self.node.wait_for_variables({"motor.right.speed"}))
        right_speed = self.node['motor.right.speed']
        left_speed = self.node['motor.left.speed']
        #aw(self.node.unlock())
        
        return [left_speed, right_speed]
    
    async def get_sensor_values(self):
        """
        Print the sensor value sensor_id print_range times, every delta_time seconds
        """
        
        aw(self.node.lock())
        aw(self.node.wait_for_variables({"prox.horizontal"})) 
        prox_sens_values = list(self.node["prox.horizontal"])
        aw(self.node.unlock())
        
        return prox_sens_values
    
    
    def get_values(self):
        """get all values

        Returns:
            _type_: _description_
        """
        aw(self.node.lock())
        aw(self.node.wait_for_variables()) 
        right_speed = self.node['motor.right.speed']
        left_speed = self.node['motor.left.speed']
        prox_sens_values = list(self.node["prox.horizontal"])
        prox_sens_values = prox_sens_values[:5]
        aw(self.node.unlock())
        return left_speed, right_speed, prox_sens_values


# Threading function 
def update_odometry(thymio):
    """Odometry part, thread that gives us the thymio position and angle based on the
    motor speed measured. Change global variable odometry.

    Args:
        thymio (thymio object): the thymio object
    """
    # take glabal variable
    global ODOMETRY, THYMIO_RADIUS, THYMIO_SPEED_CONVERTION
    
            #threading.Timer(0.5, update_odometry, args=(thymio)).start()
        
    [speed_l, speed_r] = thymio.get_speed()
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
    


threading.Thread( target=update_odometry, args=(thymio, )).start()
    
    


# check_prox_sensor thread
def check_prox_sensor(thymio):
    """This poart focus on the obstacle detection. This thread is the most prioritised one.
    

    Args:
        thymio (_type_): _description_
    """
    global PROXIMITY_SENSOR
    #print("\nHello check_sensor")
    #threading.Thread( target=check_prox_sensor, args=(thymio, )).start()
    threading.Timer(0.5, check_prox_sensor, args = (thymio)).start()
    prox_sens_values = thymio.get_sensor_values()
    
    # Stop obstacle condition
    if sum(prox_sens_values[:5]) > 10:
        thymio.set_speed([0,0])
    
    # update PROXIMITY_SENSOR
    PROXIMITY_SENSOR = prox_sens_values


def Thymio_PID_control(thymio):
    

    pass

def visual_computation():
    """This part must return the thymio position and the differents points 
    the thymio must pass by.

    Returns:
        thymio_pos_visual : np.array for size 1x2
        points_list : list of intermediate points
        ending_point : np.array of goal
    """
    # TODO : IMPLEMENT THIS PART
    return 0

def local_avoidance():
    pass





################## MAIN #################################################

def main():
    global ODOMETRY
    starting_pos = [0,0]
    starting_angle = 0
    starting_goal = [1000,1000]
    # thymio init
    thymio1 = Thymio.thymio(starting_pos, starting_angle)

    
    # call thread function
    check_prox_sensor(thymio1)
    update_odometry(thymio1)
    # create PID object
    PID = MotionControl.MotionControl()
    # initatate PID
    PID.update_angle_error(starting_angle, starting_pos, starting_goal)
  
  

    while 1:
        
        finished = False
        time.sleep(1)
        robot_angle = ODOMETRY[2]
        robot_pos = [ODOMETRY[0], ODOMETRY[1]]
        # update angle
        PID.update_angle_error(robot_angle, robot_pos, starting_goal)
        # compute PID speed
        d_time = ODOMETRY[4]
        [robot_speed_l, robot_speed_r] = PID.PID(d_time, 100, 100)
        thymio1.set_speed([robot_speed_l,robot_speed_r])
        
        print(ODOMETRY)
        if finished:
            thymio1.set_speed([0,0])
            break



def main2():
    global ODOMETRY
    starting_pos = [0,0]
    starting_angle = 0
    starting_goal = [1000,1000]
    # thymio init
    thymio1 = Thymio.thymio(starting_pos, starting_angle)

    thymio1.set_speed([20,20])
    # call thread function
    #check_prox_sensor(thymio1)
    update_odometry(thymio1)
    while 1:
        time.sleep(0.5)
        print(ODOMETRY)
    
if __name__ == '__main__':
    # Run only when this is the main file
    main2()



#%% &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

"""
TRY A NEW TECHNIQUE FOR THE ROBOT SPPED AQUISITION
"""

# computation package
import pandas as pd
import numpy as np
import math

# parralelsim package
from threading import Timer
import threading
import time

# thymio package
from tdmclient import ClientAsync
client = ClientAsync()
# create connextion w thymio
node = await client.wait_for_node()

thymio_speed = dict()
thymio_sensor = []

# create a lock for threading 
lock_ODOMETRY = threading.Lock()

# WARNING, THE ODOMETRY IS A GLOBAL VARIABLE THAT WILL BE UPDATE BY MUTIPLE THREAD 
# WE WILL HAVE TO USE A LOCK threading.lock (.realease , .aquire)
THYMIO_RADIUS = 47 # in [mm]
THYMIO_SPEED_CONVERTION = 0.3175373

ODOMETRY = [0, 0, 0, time.time(), 0]

# The function of thread that will update the odometry variable
def update_odometry():
    """Odometry part, thread that gives us the thymio position and angle based on the
    motor speed measured. Change global variable odometry.

    Args:
        thymio (thymio object): the thymio object
    """
    # take glabal variable
    global ODOMETRY, THYMIO_RADIUS, THYMIO_SPEED_CONVERTION
    while 1:
        speed = {"left_speed":node["motor.left.speed"],
                 "right_speed":node["motor.right.speed"]}
        speed_l = speed['left_speed']
        speed_r = speed['right_speed']
        #[speed_l, speed_r] = thymio.get_speed()
        # convert speed in mm/s
        speed_l = speed_l * THYMIO_SPEED_CONVERTION
        speed_r = speed_r * THYMIO_SPEED_CONVERTION
        
        [pre_pos_x, pre_pos_y, pre_angle, previous_time, d_time] = ODOMETRY
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
        lock_ODOMETRY.acquire()
        ODOMETRY = [pos_x, pos_y, angle, previous_time, d_time]
        lock_ODOMETRY.release()
        time.sleep(0.1)

def get_speed():
    """ONLY FUNCTION THAT CAN WRITE ON thymio_speed (OTHERWISE USE LOCK)
    """
    global thymio_speed
    while 1:
        # TODO try to remove the sleep
        time.sleep(0.1)
        thymio_speed.update({"left_speed":node["motor.left.speed"],
                            "right_speed":node["motor.right.speed"]})

def set_speed(left, right):
    """Set speed

    Args:
        thymio_speed (list of 2 floats): [0] for left speed, [1] for right speed

    Returns:
        none : Give thymio wheels a speed
    """
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],}

def get_sensor():
    """ONLY FUNCTION THAT CAN WRITE ON tymio_sensor (OTHERWISE USE LOCK)
    """
    global thymio_sensor
    while 1:
        # TODO try to remove the sleep
        time.sleep(0.2)
        thymio_sensor = list(node["prox.horizontal"])

#%%

# wait the connextion with variables
await node.lock()
await node.wait_for_variables()

#%%

# starting both thrad that will update the thymio data variables
threading.Thread(target=update_odometry,).start()

#%% 
threading.Thread(target=get_speed,).start()

#%% MAIN EXECUTION LOOP

import MotionControl
# initiate position angle
thymio_pos = [0, 0]
thymio_angle = 0
goal_pos = [1000, 1000]
# nomial speed
thymio_ref_speed = [100, 100]
thymio_speed = [100, 100]
PID = MotionControl.MotionControl()
PID.update_angle_error(thymio_angle, thymio_pos, goal_pos)

#%%
node.send_set_variables(set_speed(0,0))

#%%
# MAIN loop
previous_time = time.time()
time.sleep(0.1)


while False:
    # check robot angle and robot pos
    lock_ODOMETRY.acquire()
    thymio_angle = ODOMETRY[2]
    thymio_pos = [ODOMETRY[0], ODOMETRY[1]]
    lock_ODOMETRY.release()
    
    # Give PID best speed
    d_time = time.time() - previous_time
    PID.update_angle_error(thymio_angle, thymio_pos, goal_pos)
    [left_speed, right_speed] = PID.PID(d_time, thymio_ref_speed[0], thymio_ref_speed[1])
    node.send_set_variables(set_speed(left_speed, right_speed))
    previous_time = time.time()
    
    time.sleep(0.1)
    # wait a bit
    #await client.sleep(0.1)
    print(ODOMETRY)
    
#%%
# 
import math

from tdmclient import ClientAsync
client = ClientAsync()
node = await client.wait_for_node()
await node.lock()

import threading
from threading import Timer
import time

import MotionControl

acquire_data = True


thymio_data = dict()

lock_ODOMETRY = threading.Lock()
ODOMETRY = [0,0,0, time.time(), 0]




def update_odometry():
    """Odometry part, thread that gives us the thymio position and angle based on the
    motor speed measured. Change global variable odometry.

    Args:
        thymio (thymio object): the thymio object
    """
    # take glabal variable
    global ODOMETRY
    while 1:
 
        speed_l = node["motor.left.speed"]
        speed_r = node["motor.right.speed"]
        #[speed_l, speed_r] = thymio.get_speed()
        # convert speed in mm/s
        speed_l = speed_l * 0.3175373
        speed_r = speed_r * 0.3175373
        
        [pre_pos_x, pre_pos_y, pre_angle, previous_time, d_time] = ODOMETRY
        # delta time btwn last and new speed recording
        d_time = time.time() - previous_time
        # compute new angle
        d_angle = (speed_l - speed_r)/(4*47^2) * d_time
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

def set_speed(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

def get_data():
    global ODOMETRY
    while 1:
        ODOMETRY = [node["motor.left.speed"], node["motor.right.speed"]]

# Main Thread
if acquire_data:
    await node.wait_for_variables() # wait for Thymio variables values
    #threading.Thread(target=get_data,).start()
    threading.Thread(target=update_odometry,).start()
    
    #inti pid controller
    lock_ODOMETRY.acquire()
    thymio_angle = ODOMETRY[2]
    thymio_pos = [ODOMETRY[0], ODOMETRY[1]]
    lock_ODOMETRY.release()
    goal_pos = [1000,1000]
    PID = MotionControl.MotionControl()
    PID.update_angle_error(thymio_angle, thymio_pos, goal_pos)
    
    
    # variables
    d_time = 0.1
    time_last_ctrl = time.time()
    while 1:
        # time.sleep would not work here, use asynchronous client.sleep method instead
        await client.sleep(1)
        
        # get ODOMETRY values
        lock_ODOMETRY.acquire()
        print(ODOMETRY)
        thymio_angle = ODOMETRY[2]
        thymio_pos = [ODOMETRY[0], ODOMETRY[1]]
        lock_ODOMETRY.release()
        # PID controller
        PID.update_angle_error(thymio_angle, thymio_pos, goal_pos)
        d_time = time.time() - time_last_ctrl
        [left_speed, right_speed] = PID.PID(d_time, 100, 100)
        node.send_set_variables(set_speed(left_speed, right_speed))
        time_last_ctrl = time.time()
        
        manathan_dist_to_goal = abs(goal_pos[1]-thymio_pos[1]) + abs(goal_pos[0]-thymio_pos[0])
        if manathan_dist_to_goal < 200:
            node.send_set_variables(set_speed(0, 0))
            break
        
        #await client.sleep(1) # your long-running job goes here...
        #lock_ODOMETRY.acquire()
        #print(ODOMETRY)
        #lock_ODOMETRY.release()  
        #node.send_set_variables(set_speed(0, 0))

await node.unlock()

# %%
