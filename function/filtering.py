"""
FILTERING PART
    SUMMARY
    This part is there to define more prcessly where the thymio is
    for this we need the set of point of the trajectory, the wheels encoder and the gyro
    
    INPUTS
    
    OUTPUTS
    
"""
#%%
import numpy as np
import pandas as pd

import tdmclient.notebook

# %%
class obstacle:
    # newrow = [1, 2, 3] ADD ROW TO MATRIX
    # A = numpy.vstack([A, newrow])
    def __init__(self, number, type, edges_pos) -> None:
        """here are the init of the obstacle

        Args:
            number (int): to differenitaate the obstacles
            type (str): "square" "
            position (np): _description_
        """
        self.number = number
        self.type = type
        if type(edges_pos) == np.ndarray:
            self.edges_pos = edges_pos
        else:
            raise TypeError ('edges_pos in obstacle class must be a np.array')
    
    def add_edge (self, edge):
        try:
            self.edges_pos = np.vstack([self.edges_pos, edge])
        except TypeError:
            print("invalid edges (must be 1X2 array of int32)")
        

        
# class of all the data we need for the world
class World:
    def __init__(self, obstacle_l, all_objective_points, thymio) -> None:
        self.obstacle_l =  obstacle_l
        self.all_objective_points = all_objective_points
        self.thymio = thymio
        self.map_dim = np.array([480,480],dtype=np.uint32)
      
#%%      
import numpy as np
from tdmclient import ClientAsync, aw
from tdmclient.atranspiler import ATranspiler
import time
#client = ClientAsync()
#client.process_waiting_messages()


class thymio:
    def __init__(self, pos, angle) -> None:
        self.pos = pos
        self.angle = angle
        self.e_dist = np.array([0,0])
        
        self.client = ClientAsync()
        self.client.process_waiting_messages()
        self.node = aw(self.client.wait_for_node())
    
    def set_speed(self, speed_robot):
        aw(self.node.lock())
        v = {
            "motor.left.target":  [speed_robot[0]],
            "motor.right.target": [speed_robot[1]],
        }
        aw(self.node.set_variables(v))
        aw(self.node.unlock())
        
    def get_speed(self):
        aw(self.node.lock())
        aw(self.node.wait_for_variables({"motor.right.speed"}))
        right_speed =self.node['motor.right.speed']
        left_speed =self.node['motor.left.speed']
        aw(self.node.unlock())
        return [left_speed, right_speed]
    
    def get_dist(self, node, time):
        d_e_dist = time * self.get_speed()
        self.e_dist += d_e_dist
        return self.e_dist
    
    def turn(self, angle_to_turn):
        """turn the robot for a certain angle
        the k value was computed empirically
        Args:
            angle_to_turn (flaot): angle that the robot must turn in radian
        """
        aw(self.node.lock())
        right_turn = [100, 0]
        left_turn = [0,100]
        v = {
            "motor.left.target":  [speed_robot[0]],
            "motor.right.target": [speed_robot[1]],
        }
        aw(self.node.set_variables(v))
        aw(self.node.unlock())
   
    def get_d_angle(self,node, time):
        """Compute the angle the robot has turned for a given time span
        We found the k coefficiant empirically
        Args:
            node (_type_): robot node
            time (float): time in s

        Returns:
            float : the angle the robot has turned
        """
        k = 12
        d_angle = k*(self.get_speed()[0]- self.get_speed()[1])
        return d_angle
    
    def get_absolute_angle(self,node):
        pass
    
    def move_foward(self):
        pass
    
    def turn_for_100sec_pressing_butt(self):
        thymio_program_python =r"""
        @onevent
        def prox():
            global prox_horizontal, motor_left_target, motor_right_target
            prox_front = prox_horizontal[2]
            speed = -prox_front // 10
            motor_left_target = speed
            motor_right_target = speed
        """
        
        # convert program from Python to Aseba
        thymio_program_aseba = ATranspiler.simple_transpile(thymio_program_python)
        
        aw(self.node.lock())

        async def prog(self):
            aw(self.node.compile(thymio_program_aseba))
            aw(self.node.run())
        self.client.run_async_program(prog)
        
        aw(self.node.unlock())

            



    
    



class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer = None
        self.interval = interval
        self.function = function
        self.args = args
        self.kwargs = kwargs
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
# %%
pos = np.array([0,0])
angle = 0
tymio1 = thymio(pos, angle)

# %%
speed_robot = [0,0]
tymio1.set_speed(speed_robot)
#%%
print(tymio1.get_speed())  
#%%  
        



if __name__ == '__main__':
    pass


"""

import numpy as np

edge1 = np.array([2, 3], dtype=np.uint32)
edge2 = np.array([5, 6], dtype=np.uint32)

obstc1 = obstacle(1, 'square',edge1)

"""

async def prog():
    node = await client.wait_for_node()
    await node.lock()
    await node.set_variables(motors(50, 50))
    await client.sleep(2)
    await node.set_variables(motors(0, 0))
    await node.unlock()



# %%
"""code to compute the k value for rotaition simplifacation

WITH THIS PARAMETER WE FOUND THE ROTATION RATE AROUND ONE WHELL (one whell set to 0 speed)

"""
import numpy as np
from tdmclient import ClientAsync, aw
import time


client = ClientAsync()
client.process_waiting_messages()
node = aw(client.wait_for_node())
aw(node.lock())
speed_robot = [100,0]

v = {
    "motor.left.target":  [speed_robot[0]],
    "motor.right.target": [speed_robot[1]],
}
aw(node.set_variables(v))

time.sleep(10)

speed_robot = [0,0]
v = {
    "motor.left.target":  [speed_robot[0]],
    "motor.right.target": [speed_robot[1]],
}
aw(node.set_variables(v))

aw(node.unlock())
#%%

import tdmclient.notebook
await tdmclient.notebook.start()

timer_period[0]=100 #timer0 will fire 10 times per second
timer_period[1]=1000 #timer1 will fire once per second

@onevent
def buttons():
    global button_center, button_forward, button_backward, button_left, button_right, leds_top
    if button_center==1: 
        
@onevent
def timer0(): #on timer0 we toggle the color of the top LEDs
    global leds_top, toggle0
    global motor_left_target, motor_right_target
    if toggle0:
        toggle0=0
        speed_robot = [100,0]
        v = {
        "motor.left.target":  [speed_robot[0]],
        "motor.right.target": [speed_robot[1]],
        }
        aw(node.set_variables(v))
    else:
        toggle0=1
        speed_robot = [0,0]
        v = {
        "motor.left.target":  [speed_robot[0]],
        "motor.right.target": [speed_robot[1]],
        }
        aw(node.set_variables(v))
        
# %%
import tdmclient
on = False
timer_period[0] = 500
@onevent
def timer0():
    global on, leds_top
    on = not on
    if on:
        leds_top = [32, 32, 0]
    else:
        leds_top = [0, 0, 0]
# %%
from tdmclient import ClientAsync
from tdmclient.atranspiler import ATranspiler

thymio_program_python = r"""
on = False
timer_period[0] = 50000
@onevent
def timer0():
    global on, leds_top
    on = not on
    if on:
        motor_left_target = 100
    else:
        motor_left_target = 0

"""
client = ClientAsync()
client.process_waiting_messages()
node = aw(client.wait_for_node())
# convert program from Python to Aseba
thymio_program_aseba = ATranspiler.simple_transpile(thymio_program_python)
#%%
with ClientAsync() as client:
    async def prog():
        with await client.lock() as node:
            error = await node.compile(thymio_program_aseba)
            error = await node.run()
    client.run_async_program(prog)
# %%
