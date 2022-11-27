#%%
import numpy as np
import math
import matplotlib.pyplot as plt

class MotionControl:
    """
    Make a PID controller with the error being the angle error btwn thymio angle and thymio to objective angle
    """
    
    def __init__(self, Kp=70, Ki=2, Kd=0.5):
        
        #self.thymio_pos = init_thymio_pos
        #self.thymio_angle = init_thymio_angle
        #self.goal_pos = init_goal_pos
        #self.dist_frm_goal = math.sqrt((init_goal_pos[0]-init_thymio_pos[0])^2 + (init_goal_pos[0]-init_thymio_pos[0])^2)
        #self.goal_angle = math.acos(self.dist_frm_goal/(init_goal_pos[0]-init_thymio_pos[0]))
        self.thymio_angle = 0
        self.thymio_pos = [0,0]
        self.goal_pos = [0,0]
        # compute previous angle error and angle error
        self.angle_error = 0
        self.pre_angle_error = 0
        # PID parts
        self.PID_integral = 0
        self.PID_derivative = 0
        self.PID_proportional = 0
        # PID parameters
        self.Kp = Kp 
        self.Ki = Ki
        self.Kd = Kd
        self.d_time = 0.1 #240Hz = frequency for the discretization
        
        # PID output speed differential
        self.u_t = 0
        
        
        
    def update_angle_error(self, thymio_angle, thymio_pos, goal_pos):
        self.thymio_pos = thymio_pos
        self.thymio_angle = thymio_angle
        self.goal_pos = goal_pos
        
        y_ = goal_pos[1] - thymio_pos[1]
        x_ = goal_pos[0] - thymio_pos[0]

        self.goal_angle = math.atan2(y_,x_)
        
        self.angle_error = self.goal_angle - self.thymio_angle
        

    def PID(self, d_time, ref_speed_l, ref_speed_r):
        # Compute integral and derivative
        self.PID_integral = self.PID_integral + self.angle_error * d_time

        self.PID_proportional = self.angle_error
        
        self.PID_derivative = (self.angle_error - self.pre_angle_error)/d_time
        
        
        self.u_t = self.Kp*self.PID_proportional + self.Ki*self.PID_integral + self.Kd*self.PID_derivative

        
        self.pre_angle_error = self.angle_error

        # Wheel speed
        return [ref_speed_l + self.u_t, ref_speed_r - self.u_t]

    
    def plant(self,  speed_l, speed_r, pre_pos_x, pre_pos_y, pre_angle, d_time):
        
        THYMIO_SPEED_CONVERTION = 0.3175373
        THYMIO_RADIUS = 47
        # convert speed in mm/s
        speed_l = speed_l * THYMIO_SPEED_CONVERTION
        speed_r = speed_r * THYMIO_SPEED_CONVERTION

        # compute new angle
        d_angle = (speed_l - speed_r)/(4*THYMIO_RADIUS^2) * d_time
        angle = pre_angle + d_angle
        # compute new pos
        direction_x = math.cos((pre_angle + angle)/2)
        direction_y = math.sin((pre_angle + angle)/2)

        pos_x = pre_pos_x + (speed_l + speed_r)/2 * direction_x * d_time
        pos_y = pre_pos_y + (speed_l + speed_r)/2 * direction_y * d_time
        # output update ODOMETRY
        return pos_x, pos_y, angle
        
    


if __name__=='__main__':
    # TODO: an implementation to show the tymio implementation
    
    # TODO: initatte robot and goal
    init_goal_pos = [1000, 1000]
    init_robot_speed = [100,100]
    init_robot_angle = -2
    init_robot_pos = [0,0]
    
    goal_pos = [1000, 1000]
    robot_speed = [100,100]
    robot_angle = -2
    robot_pos = [0,0]
    
    d_time = 1
    thymio_trajectory = []
    
    # Create PID controller
    PID = MotionControl()
    PID.update_angle_error(robot_angle, robot_pos, goal_pos)
    # TODO: loop
    loop = 0
    while True:
        loop += 1
        goal_achieved = False
        PID.update_angle_error(robot_angle, robot_pos, goal_pos)
        
        [robot_speed[0], robot_speed[1]] = PID.PID(d_time, 100, 100)
        [robot_pos[0], robot_pos[1], robot_angle] = PID.plant(robot_speed[0], robot_speed[1], robot_pos[0], robot_pos[1], robot_angle, d_time)
        thymio_trajectory.append((robot_pos[0], robot_pos[1]))
        manathan_dist_to_goal = abs(goal_pos[1]-robot_pos[1]) + abs(goal_pos[0]-robot_pos[0])
        if manathan_dist_to_goal < 200 or loop > 300:
            break
        
    
    # TODO: compute PID speed
    # TODO: set speed and get new position and angle value
    
    # plot trajectorie
    plt.scatter(*zip(*thymio_trajectory))
    plt.plot([0,1000], [0,1000], color = 'red', linestyle = 'solid')
    plt.plot([0,100*math.cos(init_robot_angle)], [0, 100 * math.sin(init_robot_angle)], color = 'green')
    plt.title('Thymio PID direction')
#%%
    