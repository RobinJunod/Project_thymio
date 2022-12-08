import numpy as np
import math


class MotionControl:
    def __init__(self, Kp=70, Ki=2, Kd=0.5):
        """Make a PID controller with the error being the angle error btwn thymio angle and thymio to objective angle

        Args:
            Kp (int, optional): proportional coefficient. Defaults to 70.
            Ki (int, optional): integral coefficient. Defaults to 2.
            Kd (float, optional): derivative coefficient. Defaults to 0.5.
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
        self.d_time = 0.1 #240Hz = frequency for the discretization
        
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
                    
        self.angle_error = self.goal_angle - self.thymio_angle
        if self.angle_error > np.pi :
            self.angle_eror = 2*np.pi - self.angle_error
        elif self.angle_error < -np.pi:
            self.angle_error = self.angle_error + 2*np.pi
        
        
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
        if d_time > 0:
            self.PID_integral = self.PID_integral + self.angle_error * d_time

            self.PID_derivative = (self.angle_error - self.pre_angle_error)/d_time
        else:
            self.PID_integral = 0

            self.PID_derivative = 0


        self.PID_proportional = self.angle_error
        
        self.u_t = self.Kp*self.PID_proportional + self.Ki*self.PID_integral + self.Kd*self.PID_derivative

        
        self.pre_angle_error = self.angle_error

        # Wheel speed
        return [ref_speed_l - self.u_t, ref_speed_r + self.u_t]

    
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
        d_angle = (speed_l - speed_r)/(4*THYMIO_RADIUS^2) * d_time
        angle = pre_angle + d_angle
        # compute new pos
        direction_x = math.cos((pre_angle + angle)/2)
        direction_y = math.sin((pre_angle + angle)/2)

        pos_x = pre_pos_x + (speed_l + speed_r)/2 * direction_x * d_time
        pos_y = pre_pos_y + (speed_l + speed_r)/2 * direction_y * d_time
        # output update ODOMETRY
        return pos_x, pos_y, angle
        
    