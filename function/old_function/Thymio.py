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

from tdmclient import ClientAsync, aw
from tdmclient.atranspiler import ATranspiler

import time

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
    

if __name__=='__main__':
    pos = [0,0]
    angle = 0
    thymio1 = thymio(pos, angle)
    thymio1.set_speed([100,0])
    time.sleep(3)
    print(thymio1.get_speed())
    
    
#%%
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

"""
       thymio_program_python =r"
        @onevent
        def prox():
            global prox_horizontal, motor_left_target, motor_right_target
            prox_front = prox_horizontal[2]
            speed = -prox_front // 10
            motor_left_target = speed
            motor_right_target = speed
        "
        
        # convert program from Python to Aseba
        thymio_program_aseba = ATranspiler.simple_transpile(thymio_program_python)
        
        aw(self.node.lock())

        async def prog(self):
            aw(self.node.compile(thymio_program_aseba))
            aw(self.node.run())
        self.client.run_async_program(prog)
        
        aw(self.node.unlock()
"""