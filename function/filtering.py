import numpy as np

q_angle = 0.08
q_pos = 10
r_angle = 0.01
r_pos = 2

##### Angle filtering

def kalmanFilterAngle(mu_est_angle, y_angle, Sigma_angle):

    Sigma_angle = Sigma_angle + q_angle
    i_t = y_angle - mu_est_angle
    if i_t > np.pi :
        i_t = 2*np.pi - i_t
    elif i_t < -np.pi:
            i_t = i_t + 2*np.pi
    K_t = Sigma_angle / (Sigma_angle+r_angle)
    mu_angle = mu_est_angle + K_t * i_t

    if mu_angle >= np.pi:
        mu_angle = mu_angle - 2*np.pi
    elif mu_angle <= -np.pi:
        mu_angle = 2*np.pi - mu_angle
    else:
        mu_angle = mu_angle
    Sigma_angle = (1-K_t)*Sigma_angle

    return mu_angle, Sigma_angle

def kalmanFilterPos(mu_est_x, mu_est_y, y_x, y_y, Sigma_pos):

    Sigma_pos = Sigma_pos + q_pos
    i_t_x = y_x - mu_est_x
    i_t_y = y_y - mu_est_y
    K_t = Sigma_pos / (Sigma_pos + r_pos)
    mu_x = mu_est_x + K_t * i_t_x
    mu_y = mu_est_y + K_t * i_t_y
    Sigma_pos = (1-K_t)*Sigma_pos

    return mu_x, mu_y, Sigma_pos