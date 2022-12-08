import cv2
import numpy as np
import math as m
import time

# CONSTANT INIT

k_sobel = 5

MAP_WIDTH = 1070 # mm
THYMIO_WIDTH = 110 # mm

# Set the Lower and Higher range value of color for filter
#RED_L = np.array([105,75,129],np.uint8)  
#RED_H = np.array([179,160,255],np.uint8)
RED_L= np.array([133,50,130]) 
RED_H = np.array([179,255,255])

#GREEN_L = np.array([0, 60, 90],np.uint8)
#GREEN_H = np.array([87, 255, 255],np.uint8)
GREEN_L = np.array([0, 0, 140],np.uint8)
GREEN_H = np.array([90, 90, 255],np.uint8)

#BLUE_L = np.array([0, 79, 154],np.uint8)
#BLUE_H = np.array([156, 255, 255],np.uint8)
BLUE_L = np.array([0, 90, 120],np.uint8)
BLUE_H = np.array([110, 255, 255],np.uint8)

YELLOW_L = np.array([0, 0, 158],np.uint8)
YELLOW_H = np.array([179, 240, 255],np.uint8)


half_thymio_pix = 38

    

def sobelFilter(img, k):
    filtered_img = cv2.bilateralFilter(img,3,75,75)

    sobelx = cv2.Sobel(filtered_img,cv2.CV_64F,1,0,k)
    sobely = cv2.Sobel(filtered_img,cv2.CV_64F,0,1,k)
    sobel = cv2.sqrt(cv2.addWeighted(cv2.pow(sobelx, 2.0), 1.0, cv2.pow(sobely, 2.0), 1.0, 0.0))

    return sobel

def colorFilter(img, lower_range, upper_range):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,lower_range,upper_range) # Create a mask with range
    filtered_img = cv2.bitwise_and(img,img,mask = mask)  # Performing bitwise and operation with mask in img variable

    return hsv, mask, filtered_img

def mapTransform(img,YELLOW_L, YELLOW_H):
	hsv, mask, field = colorFilter(img,YELLOW_L, YELLOW_H)
	
	field_gray = cv2.cvtColor(field, cv2.COLOR_BGR2GRAY)
	# noise
	filtered_img = cv2.bilateralFilter(field_gray,3,75,75)
	# binary
	binary_img = cv2.threshold(filtered_img, 20, 255, cv2.THRESH_BINARY_INV)[1]

	# find contours of MAP
	contours, _ = cv2.findContours(binary_img.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	approx_cont = []
	circle = []
	for cnt in contours:
		approx = cv2.approxPolyDP(cnt, 0.03*cv2.arcLength(cnt, True), True)
		area = cv2.contourArea(approx)
		# Criteria on the area to get only the external frame and not the obstacle or thymio if color filter did not work fine
		if (binary_img.shape[0]*binary_img.shape[1]/2 < area < binary_img.shape[0]*binary_img.shape[1]):
			approx_cont.append(approx.reshape(-1, 2))          
    
    # Rectify
	pts1 = approx_cont[0]
    #Target Points
	t1 = pts1[0,0] #corner top left
	t2 = pts1[0,1]
	r1 = pts1[3,0] #corner top right
	r2 = pts1[3,1]
	b1 = pts1[2,0] #corner bottom right
	b2 = pts1[2,1]
	width = m.floor(m.sqrt((r1 - t1) ** 2 + (r2 - t2) ** 2))
	height = m.floor(m.sqrt((b1 - r1) ** 2 + (b2 - r2) ** 2))
	pts2 = np.float32([[0, 0], [0, height], [width, height], [width, 0]])
	M = cv2.getPerspectiveTransform(pts1.astype(np.float32), pts2)
	dst = cv2.warpPerspective(img, M, (width, height))

	## 3. Rescaling image (between width and height)
	rescaled = dst[0:height, 0:width]
	return rescaled, M, width, height

def mm2pixRatio(img):
	# Conversion mm to pixel
	pix_width = img.shape[1]
	pix_per_mm = pix_width / MAP_WIDTH

	return pix_per_mm

def rescale_borders(img):
	map_rescaled = np.copy(img)
	map_rescaled = map_rescaled[10:img.shape[0]-10, 10:img.shape[1]-10]

	return map_rescaled
	

def findThymio(img, lower_range, upper_range, k_small, k_big, THRESH):
	_, _, thymio = colorFilter(img,lower_range,upper_range)
	thymio_gray = cv2.cvtColor(thymio, cv2.COLOR_BGR2GRAY)

	# Find big round by eroding s.t. small round disapear
	kernelRound25 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(k_big,k_big))
	thymio_big = cv2.erode(thymio_gray, kernelRound25, iterations=1)
	tbr = cv2.dilate(thymio_big, kernelRound25, iterations=1)
	# Find small Thymio
	tsr = thymio_gray - tbr
	kernelRound5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(k_small,k_small))
	filt_tsr = cv2.erode(tsr, kernelRound5, iterations=1)

	tmp = cv2.threshold(tbr, THRESH, 255, cv2.THRESH_BINARY_INV)[1]
	tmp2 = cv2.threshold(filt_tsr, THRESH, 255, cv2.THRESH_BINARY_INV)[1]

	idx = np.where(tmp==0)
	if not idx:
		BR = [np.nan,np.nan]
	else:
		BR = [np.mean(idx[1]),np.mean(idx[0])]
	
	idx2 = np.where(tmp2==0)
	if not idx2:
		SR = [np.nan,np.nan]
	else:
		SR = [np.mean(idx2[1]),np.mean(idx2[0])]

	posX = BR[0] # Big round is at the center of rotation of the wheels so give that as thymio position
	posY = BR[1]
	pos = [posX,posY]
	dx = SR[0]-BR[0]
	dy = -(SR[1]-BR[1])
	
	angle = m.atan2(dy,dx)

	return pos, angle
	
def findGoal(img,lower_range,upper_range,TRESH_L,TRESH_H):
	hsv, mask, filt_img = colorFilter(img,lower_range,upper_range)
	goal_gray = cv2.cvtColor(filt_img, cv2.COLOR_BGR2GRAY)
	tmp = cv2.threshold(goal_gray, TRESH_L, TRESH_H, cv2.THRESH_BINARY_INV)[1]
	tmp = cv2.GaussianBlur(tmp, (5,5), 5,5)
	idx = np.where(tmp==0)
	GOAL = [np.mean(idx[1]),np.mean(idx[0])]
	return GOAL

def findObst(img,mm2pix,lower_range,upper_range,TRESH_L,TRESH_H):
	hsv, mask, filt_img = colorFilter(img,lower_range,upper_range)
	obst_gray = cv2.cvtColor(filt_img, cv2.COLOR_BGR2GRAY)
	_, binary_img = cv2.threshold(obst_gray, TRESH_L, TRESH_H, cv2.THRESH_BINARY)
	# find contours of obstacles
	contours, _ = cv2.findContours(binary_img.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	approx_array = []
	circle = []
	for cnt in contours:
			approx = cv2.approxPolyDP(cnt, 0.03 * cv2.arcLength(cnt, True), True)
			area = cv2.contourArea(approx)
			if 1000 < area < 10000:
				approx_array.append(approx.reshape(-1, 2))
	    

	exp_cnt = np.copy(approx_array)
	for i, cnt in enumerate(approx_array):
		for j, corner in enumerate(cnt):
			e1 = cnt[(j-1)%cnt.shape[0]]-corner
			e2 = cnt[(j+1)%cnt.shape[0]]-corner
			bisector = (e1 / np.linalg.norm(e1) + e2 / np.linalg.norm(e2))
			exp_cnt[i][j] = corner - (mm2pix*THYMIO_WIDTH*0.75) * bisector / np.linalg.norm(bisector)

	exp_img = cv2.drawContours(binary_img.copy(), exp_cnt, -1, 255, thickness=cv2.FILLED)

	contours_exp, _ = cv2.findContours(exp_img.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	approx_array_exp = []
	
	for cnt in contours_exp:
		approx = np.array([], dtype=object)
		approx = cv2.approxPolyDP(cnt, 0.03 * cv2.arcLength(cnt, True), True)
		area = cv2.contourArea(approx)
		if 5000 < area < 80000:
			approx_array_exp.append(approx.reshape(-1, 2))

	exp_img_app = cv2.drawContours(exp_img.copy(), approx_array_exp, -1, 255, thickness=cv2.FILLED)

	return approx_array_exp, exp_img_app

############### Functions used in MAIN ############################################################

def map_init(img):
	### Map rescaling and rectifying
	recMap, M, IMGwidth, IMGheight = mapTransform(img,YELLOW_L, YELLOW_H)
	mm2pix = mm2pixRatio(recMap)
	rescMap = rescale_borders(recMap)
	mapsize = [rescMap.shape[1],rescMap.shape[0]]
    
	### Find Goal
	goal_coords = findGoal(rescMap,RED_L,RED_H,20,255)
    
    ### Find Obstacles
	obst_coords, image = findObst(rescMap,mm2pix,BLUE_L,BLUE_H,20,10)
    
    ### Find Thymio
	t_coords, t_angle = findThymio(rescMap, GREEN_L, GREEN_H,3,20,20)
    
	return rescMap, M, IMGwidth, IMGheight, mapsize, mm2pix, goal_coords, obst_coords, t_coords, t_angle

def updateThymioPos(img, M, width, height):
    ### Map rescaling and rectifying
    dst = cv2.warpPerspective(img, M, (width, height))
    recMap = dst[0:height, 0:width]
    rescMap = rescale_borders(recMap)
    ### Find Thymio
    t_coords, t_angle = findThymio(rescMap, GREEN_L, GREEN_H,3,20,20)
    
    return rescMap, t_coords, t_angle

