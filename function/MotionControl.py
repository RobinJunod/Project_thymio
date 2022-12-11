#%%
import numpy as np
import math
import matplotlib.pyplot as plt

class MotionControl:
    def __init__(self, Kp=70, Ki=2, Kd=0.4):
        """Make a PID controller with the error being the angle error btwn thymio angle and thymio to objective angle
        Args:
            Kp (int, optional): proportional coefficient. Defaults to 70.
            Ki (int, optional): integral coefficient. Defaults to 2.
            Kd (float, optional): derivative coefficient. Defaults to 0.4.
        """
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
        #self.d_time = 0.1 #240Hz = frequency for the discretization
        
        # PID output speed differential
        self.u_t = 0
        
        
        
    def update_angle_error(self, thymio_angle, thymio_pos, goal_pos):
        """Compute the angle error. 
        The angle error is the angle btwn the robot direction and the robot-to-goal angle.
        The thymio angle is already known but we need to compute the robot-to-goal angle using the position. 
        
        Args:
            thymio_angle (_type_): _description_
            thymio_pos (_type_): _description_
            goal_pos (_type_): _description_
        """
        self.thymio_pos = thymio_pos
        self.thymio_angle = thymio_angle
        self.goal_pos = goal_pos
        
        y_ = thymio_pos[1] - goal_pos[1]
        x_ = goal_pos[0] - thymio_pos[0]

        # function to get the angle without /0 problem, and good computation speed
        self.goal_angle = math.atan2(y_,x_)   
        # compute error angle
        self.angle_error = self.goal_angle - self.thymio_angle
        
        # put error angle btwn -pi and pi
        if self.angle_error > 3.1415:
            self.angle_error = self.angle_error - 6.283
        elif self.angle_error < - 3.1415:
            self.angle_error = self.angle_error + 6.283
            
        

    def PID(self, d_time, ref_speed_l, ref_speed_r):
        """PID implementation.
        Args:
            d_time (float): time btwn the last and new speed allocation
            ref_speed_l (float): left reference speed at which the robot 
                                 is supposed to go in a straight line
            ref_speed_r (float): right reference speed at which the robot 
                                 is supposed to go in a straight line
        Returns:
            float list: motors speed
        """
        # Compute integral and derivative
        if d_time == 0:
            self.PID_derivative = 0
        else:
            self.PID_derivative = (self.angle_error - self.pre_angle_error)/d_time

        self.PID_integral = self.PID_integral + self.angle_error * d_time
        self.PID_proportional = self.angle_error
        
        self.u_t = self.Kp*self.PID_proportional + self.Ki*self.PID_integral + self.Kd*self.PID_derivative
        
        self.pre_angle_error = self.angle_error

        # Wheel speed
        return [int(ref_speed_l - self.u_t), int(ref_speed_r + self.u_t)]

    
    def plant(self,  speed_l, speed_r, pre_pos_x, pre_pos_y, pre_angle, d_time):
        """This function is used to simulate the PID controller. This can help us tuning the PID.
        Mainly used for the graphic. It is used to set the new motor speed found by the PID. This 
        part is like the odomerty part expect that the robot has a perfect(no noise) behaviour.
        Args:
            speed_l (float): motor left speed
            speed_r (float): motor right speed
            pre_pos_x (float): previous robot x position
            pre_pos_y (float): previous robot y position
            pre_angle (float): previous robot angle
            d_time (float): time between 2 values
        Returns:
            float list: position and angle of the robot
        """
        THYMIO_SPEED_CONVERTION = 0.3175373
        THYMIO_RADIUS = 47
        # convert speed in mm/s
        speed_l = speed_l * THYMIO_SPEED_CONVERTION
        speed_r = speed_r * THYMIO_SPEED_CONVERTION

        # compute new angle
        d_angle = (speed_r - speed_l)/(2*THYMIO_RADIUS) * d_time
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
    
    # initatte robot and goal
    init_goal_pos = [1000, 1000]
    init_robot_speed = [100,100]
    init_robot_angle = -4
    init_robot_pos = [0,0]
    
    goal_pos = [1000, 1000]
    robot_speed = [100,100]
    robot_angle = -4
    robot_pos = [0,0]
    
    d_time = 1
    thymio_trajectory = []
    
    # Create PID controller
    PID = MotionControl()
    PID.update_angle_error(init_robot_angle, init_robot_pos, init_goal_pos)
    # loop
    loop = 0
    while True:
        loop += 1
        goal_achieved = False
        # compute error
        PID.update_angle_error(robot_angle, robot_pos, goal_pos)
        # compute PID speed
        [robot_speed[0], robot_speed[1]] = PID.PID(d_time, 100, 100)
        # set speed and get new position and angle value
        [robot_pos[0], robot_pos[1], robot_angle] = PID.plant(robot_speed[0], robot_speed[1], robot_pos[0], robot_pos[1], robot_angle, d_time)
 
        thymio_trajectory.append((robot_pos[0], robot_pos[1]))
        manathan_dist_to_goal = abs(goal_pos[1]-robot_pos[1]) + abs(goal_pos[0]-robot_pos[0])
        if manathan_dist_to_goal < 200 or loop > 100:
            break
    
    # plot trajectorie
    plt.scatter(*zip(*thymio_trajectory))
    plt.plot([0,1000], [0,1000], color = 'red', linestyle = 'solid')
    plt.plot([0,100*math.cos(init_robot_angle)], [0, 100 * math.sin(init_robot_angle)], color = 'green')
    plt.title('Thymio PID direction')
    
# %%
