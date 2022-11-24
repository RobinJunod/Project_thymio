#%%
import numpy as np
#import pandas as pd


from tdmclient import ClientAsync, aw
from tdmclient.atranspiler import ATranspiler

import time

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

#client = ClientAsync()
#client.process_waiting_messages()


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
        right_speed = self.node['motor.right.speed']
        left_speed = self.node['motor.left.speed']
        aw(self.node.unlock())
        return [left_speed, right_speed]
    
    def get_sensor_values(self):
        """
        Print the sensor value sensor_id print_range times, every delta_time seconds
        """
        aw(self.node.lock())
        aw(self.node.wait_for_variables({"prox.horizontal"})) 
        prox_sens_values = list(self.node["prox.horizontal"])
        aw(self.node.unlock())
        return prox_sens_values
    
    
    def get_dist(self, previous_time):
        d_time = time.time()- previous_time
        d_e_dist = d_time * self.get_speed()
        self.e_dist += d_e_dist
        return self.e_dist
    
    def turn(self, angle_to_turn, direction):
        """turn the robot for a certain angle
        the k value was computed empirically
        Args:
            angle_to_turn (flaot): angle that the robot must turn in radian
        """
        aw(self.node.lock())
        right_turn = [100, 0]
        left_turn = [0,100]
        if direction == 'right':
            v = {
            "motor.left.target":  [right_turn[0]],
            "motor.right.target": [right_turn[1]],
            }       
        elif direction == 'left':
            v = {
                "motor.left.target":  [left_turn[0]],
                "motor.right.target": [left_turn[1]],
            }
        else:
            raise ValueError('thymio  turn must be right or left')
        
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

            

if __name__=='__main__':
    pos = [0,0]
    angle = 0
    thymio1 = thymio(pos, angle)
    thymio1.set_speed([100,0])
    time.sleep(3)
    print(thymio1.get_speed())
    
    
    
"""
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
"""