import cv2
import numpy as np
import math




def visualisation(rescMap, POS_VISIONX, POS_VISIONY, obst_coords, ANGLE_VISION, shortestpath, goal_coordsX, goal_coordsY):
    image = cv2.circle(rescMap, (math.floor(POS_VISIONX),math.floor(POS_VISIONY)), 30, (0,255,0), 2)
    x_g = math.floor(goal_coordsX)
    y_g = math.floor(goal_coordsY)
    x_t = math.floor(POS_VISIONX)
    y_t = math.floor(POS_VISIONY)
    image = cv2.circle(image, (x_g,y_g), 30, (0,0,255), 2)
    for obst in obst_coords:
        pts = obst
        pts = pts.reshape((-1,1,2))
        image = cv2.polylines(image,[pts.astype(np.int32)],True,(255,0,0))
    image = cv2.line(image,(x_t,y_t),(math.floor(x_t + 30*math.cos(ANGLE_VISION)),math.floor(y_t-30*math.sin(ANGLE_VISION))),(0,0,0),5)
    image = cv2.circle(image, (x_t,y_t), 30, (0,255,0), 2)
    image = cv2.polylines(image, np.int32([np.array(shortestpath).reshape((-1, 1, 2))]), False, (200, 0, 255), 3)
    resized_up = cv2.resize(image, [2*image.shape[1],2*image.shape[0]], interpolation= cv2.INTER_LINEAR)
    cv2.imshow("Display window", resized_up)

def visuDuringRun(rescMap, POS_VISIONX, POS_VISIONY, obst_coords, newANGLE, Sigma_angle, shortestpath, goal_coordsX, goal_coordsY):
    image = cv2.circle(rescMap, (math.floor(POS_VISIONX),math.floor(POS_VISIONY)), 30, (0,255,0), 2)
    x_g = math.floor(goal_coordsX)
    y_g = math.floor(goal_coordsY)
    x_t = math.floor(POS_VISIONX)
    y_t = math.floor(POS_VISIONY)
    image = cv2.circle(image, (x_g,y_g), 30, (0,0,255), 2)
    for obst in obst_coords:
        pts = obst
        pts = pts.reshape((-1,1,2))
        image = cv2.polylines(image,[pts.astype(np.int32)],True,(255,0,0))
    
    image = cv2.line(image,(x_t,y_t),(math.floor(x_t + 30*math.cos(newANGLE+Sigma_angle)),math.floor(y_t-30*math.sin(newANGLE+Sigma_angle))),(0,0,255),5)
    image = cv2.line(image,(x_t,y_t),(math.floor(x_t + 30*math.cos(newANGLE-Sigma_angle)),math.floor(y_t-30*math.sin(newANGLE-Sigma_angle))),(0,0,255),5)

    image = cv2.circle(image, (x_t,y_t), 30, (0,255,0), 2)
    image = cv2.polylines(image, np.int32([np.array(shortestpath).reshape((-1, 1, 2))]), False, (200, 0, 255), 3)
    resized_up = cv2.resize(image, [2*image.shape[1],2*image.shape[0]], interpolation= cv2.INTER_LINEAR)
    cv2.imshow("Display window", resized_up)
                    