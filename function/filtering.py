import numpy as np

q_angle = 0.15
r_angle = 0.05

##### Angle filtering

def kalmanFilterAngle(mu_est_angle, y_angle, Sigma_angle):

    Sigma_angle = Sigma_angle + q_angle
    i_t = y_angle - mu_est_angle
    K_t = Sigma_angle / (Sigma_angle+r_angle)
    mu_angle = mu_est_angle + K_t * i_t

    return mu_angle, Sigma_angle