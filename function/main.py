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

# Global constants
THYMIO_RADIUS = 47 # in [mm]
THYMIO_SPEED_CONVERTION = 0.3175373


# init ODOMERTRY
ODOMETRY = [0, 0, 0, time.time()]

################################# THREADS #######################################



# Threading function 
def update_odometry(thymio):
    """Odometry part, thread that gives us the thymio position and angle based on the
    motor speed measured. Change global variable odometry.

    Args:
        thymio (thymio object): the thymio object
    """
    # take glabal variable
    global ODOMETRY, THYMIO_RADIUS, THYMIO_SPEED_CONVERTION

    threading.Timer(0.5, update_odometry, [thymio]).start()
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

# check_prox_sensor thread
def check_prox_sensor(thymio):
    """This poart focus on the obstacle detection. This thread is the most prioritised one.
    

    Args:
        thymio (_type_): _description_
    """
    global PROXIMITY_SENSOR
    #print("\nHello check_sensor")
    threading.Timer(0.5, check_prox_sensor, [thymio]).start()
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
    starting_angle = [0]
    starting_goal = [1000,1000]
    # thymio init
    thymio1 = Thymio.thymio(starting_pos, starting_angle)

    
    # call thread function
    check_prox_sensor(thymio1)
    update_odometry(thymio1)
    # create PID object
    PID = MotionControl()
    # initatate PID
    PID.update_angle_error(starting_angle, starting_pos, starting_goal)
  
  

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

