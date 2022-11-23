#%%

# basic computation library
import numpy as np

# package for thymio
from tdmclient import ClientAsync, aw
from tdmclient.atranspiler import ATranspiler

# package for threading
import time
import asyncio
import threading

# Created files
import Thymio

# Threading function 
def compute_encoder(thymio, dist_r, dist_l, previous_time):
    print('hello encoder\n')
    [speed_l, speed_r] = thymio.get_speed()
    d_time = time.time() - previous_time
    
    d_dist_r = d_time * speed_r
    d_dist_l = d_time * speed_l
    dist_r = dist_r + d_dist_r
    dist_l = dist_l + d_dist_l
    
    return dist_l, dist_l, d_dist_l, d_dist_r


def check_prox_sensor(thymio):
    print("check_sensor\n")
    prox_sens_values = thymio.get_sensor_values()
    print(prox_sens_values)
    return prox_sens_values

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

def main():
    inital_states = visual_computation()
    starting_pos = [0,0]
    starting_angle = []
    
    # thymio init
    thymio1 = Thymio.thymio(starting_pos, starting_angle)
    
    goal_reached = False
    dist_r =0
    dist_l =0
    previous_time = 10
    loop =0
    
    while 1:
        loop +=1
        check_prox_sensor(thymio1)
        time.sleep(5)
        if goal_reached == True:
            break
        if loop == 100:
            break
    
main()  


#%%
#def hello():
#    print("hello, world")
#
#t = threading.Timer(2.0, hello)
#t.start()  # after 30 seconds, "hello, world" will be printed
#
#print('hi\n')








"""
async def print_sensor_values(sensor_id, print_range=10, delta_time=0.2):
    
    Print the sensor value sensor_id print_range times, every delta_time seconds
    
    await node.wait_for_variables({str(sensor_id)})
    for i in range(print_range):
        print(list(node[sensor_id]))
        await client.sleep(delta_time)
        
await print_sensor_values('prox.ground.reflected')
# %%


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
