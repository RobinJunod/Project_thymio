
"""
TRY A NEW TECHNIQUE FOR THE ROBOT SPPED AQUISITION
"""
    
#%%

import math

import threading
from threading import Timer
import time

from tdmclient import ClientAsync, aw
client = ClientAsync()
node = aw(client.wait_for_node())
aw(node.lock()) 

import MotionControl


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
ODOMETRY = [0,0,0, time.time(), 0]
def thread_update_odometry():
    """                                    
    This thread is executed in continus and is there to update the global variable ODOMERTY
    Args:
        thymio (): will use the global variables node and ODOMETRY
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
        
Thread_odomerty = threading.Thread(target=thread_update_odometry,)


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
        prox_sens_values = list(node["prox.horizontal"])
        # Stop obstacle condtion
        if sum(prox_sens_values[:5]) > 10:
            pass
        # update PROXIMITY_SENSOR
        lock_PROX_SENSOR.acquire()
        PROX_SENSOR = prox_sens_values
        lock_PROX_SENSOR.release()
        time.sleep(0.1)

Thread_sensor = threading.Thread(target=thread_get_sensor,)

# &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&


def set_speed(left, right):
    """
    small function to be used with node.send_set_variables()
    """
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }
    

    

def get_data():
    """Test function TODO: remove it
    """
    global ODOMETRY
    while 1:
        ODOMETRY = [node["motor.left.speed"], node["motor.right.speed"]]

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
    aw(node.wait_for_variables()) # wait for Thymio variables values
    #threading.Thread(target=get_data,).start()
    #threading.Thread(target=update_odometry,).start()
    
    # Start Threads
    Thread_odomerty.start()
    Thread_sensor.start()
    
    
    #init pid controller
    # TODO : add the computation of the thymio angle and posistion
    lock_ODOMETRY.acquire()
    thymio_angle = ODOMETRY[2]
    thymio_pos = [ODOMETRY[0], ODOMETRY[1]]
    lock_ODOMETRY.release()
    
    # TODO : add the computation of the shortest path here
    goals_list = [(1000,1000), (2000,1000)]
    n_goal = 0
    goal_pos = list(goals_list[n_goal])
    
    PID = MotionControl.MotionControl()
    PID.update_angle_error(thymio_angle, thymio_pos, goal_pos)
    
    
    # variables
    d_time = 0.1
    time_last_ctrl = time.time()
    while 1:
        # time.sleep would not work here, use asynchronous client.sleep method instead
        aw(client.sleep(1)) 
        
        # get ODOMETRY values
        # take variables from GV ODOMERTY
        lock_ODOMETRY.acquire()
        print(ODOMETRY)
        thymio_angle = ODOMETRY[2]
        thymio_pos = [ODOMETRY[0], ODOMETRY[1]]
        lock_ODOMETRY.release()
        
        lock_PROX_SENSOR.acquire()
        print(PROX_SENSOR)
        lock_PROX_SENSOR.release()
        
        # PID controller
        PID.update_angle_error(thymio_angle, thymio_pos, goal_pos)
        d_time = time.time() - time_last_ctrl
        [left_speed, right_speed] = PID.PID(d_time, 100, 100)
        node.send_set_variables(set_speed(left_speed, right_speed))
        time_last_ctrl = time.time()
        
        # TODO Kalman filter (give a new value to the odomerty)
        
        
        # TODO: reaplce manathan distance by L2 norm
        manathan_dist_to_goal = abs(goal_pos[1]-thymio_pos[1]) + abs(goal_pos[0]-thymio_pos[0])
        if manathan_dist_to_goal < 200:
            # TODO: change goal position arrcording to goal list
            n_goal += 1
            if n_goal < len(goals_list):
                goal_pos = list(goals_list[n_goal])
            else:
                # thymio reached destination
                node.send_set_variables(set_speed(0, 0))
                break

aw(node.unlock())

# %%