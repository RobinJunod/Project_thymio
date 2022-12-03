import numpy as np
#import utils
from tdmclient import ClientAsync, aw

prox_and_memory = np.zeros(7)
k_ann = 1500
offset_ann = 150

#def compute_speed(vision, actual_pos, target_pos, min_angle, offset_speed, k_forward, k_rot, is_near_checkpoint):
#    wheel_radius = utils.WHEEL_RADIUS_MM * vision.mm2px
#    robot_width  = utils.THYMIO_WIDTH_MM * vision.mm2px
#    delta_pos = target_pos - actual_pos[0:2]
#    alpha = -actual_pos[2] + np.arctan2(delta_pos[1], delta_pos[0])
#    alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
#    if (abs(alpha) > min_angle) & is_near_checkpoint:
#        return [int( k_rot * alpha * robot_width / 2 / wheel_radius),
#                int(-k_rot * alpha * robot_width / 2 / wheel_radius)]
#    else:
#        speed_left  = offset_speed + k_forward * alpha
#        speed_right = offset_speed - k_forward * alpha
#        speed = [int(speed_left), int(speed_right)]
#        return speed

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

def set_speed(speed_robot, node):
    v = {
        "motor.left.target":  [speed_robot[0]],
        "motor.right.target": [speed_robot[1]],
    }
    aw(node.set_variables(v))


def get_speed(node):
    right_speed = node['motor.right.speed']
    left_speed = node['motor.left.speed']
    return [left_speed, right_speed]

