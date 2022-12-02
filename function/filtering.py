import numpy as np

q_angle = 0.15
q_pos = 10
r_angle = 0.05
r_pos = 5

##### Angle filtering

def kalmanFilterAngle(mu_est_angle, y_angle, Sigma_angle):

    Sigma_angle = Sigma_angle + q_angle
    i_t = y_angle - mu_est_angle
    K_t = Sigma_angle / (Sigma_angle+r_angle)
    mu_angle = mu_est_angle + K_t * i_t

    return mu_angle, Sigma_angle

def kalmanFilterPos(mu_est_x, mu_est_y, y_x, y_y, Sigma_pos):

    Sigma_pos = Sigma_pos + q_pos
    i_t_x = y_x - mu_est_x
    i_t_y = y_y - mu_est_y
    K_t = Sigma_pos / (Sigma_pos + r_pos)
    mu_x = mu_est_x + K_t * i_t_x
    mu_y = mu_est_y + K_t * i_t_y

    return mu_x, mu_y, Sigma_pos