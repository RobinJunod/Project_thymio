import cv2
import numpy as np
import math as m

# CONSTANT INIT

MAP_WIDTH = 1070 # mm
THYMIO_WIDTH = 110 # mm
global pix_per_mm # ratio between pixel and mm

# Lower and Higher range value of color for filter

RED_L= np.array([133,50,130]) 
RED_H = np.array([179,255,255])

GREEN_L = np.array([0, 0, 140],np.uint8)
GREEN_H = np.array([90, 90, 255],np.uint8)

BLUE_L = np.array([0, 90, 120],np.uint8)
BLUE_H = np.array([110, 255, 255],np.uint8)

YELLOW_L = np.array([0, 0, 158],np.uint8)
YELLOW_H = np.array([179, 240, 255],np.uint8)



def colorFilter(img, lower_range, upper_range):
	"""
	This function is used to filter the color of an image. The pixels not in the color range will turn black

        Args:
			img: image to filter
            lower range: [Hmin,Smin,Vmin] lower HSV value for the chosen color
			upper range: [Hmax,Smax,Vmax] upper HSV value for the chosen color
            
        Returns:
            filtered_img: filtered image
    """
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv,lower_range,upper_range) # Create a mask with range
	filtered_img = cv2.bitwise_and(img,img,mask = mask)  # Performing bitwise and operation with mask in img variable

	return hsv, mask, filtered_img

def mapTransform(img,lower_range,upper_range):
	"""
	This function is used to find the edges of the map and correct the perspective

        Args:
			img: image taken from the camera
            lower range: [Hmin,Smin,Vmin] lower HSV value for the color of the frame 
			upper range: [Hmax,Smax,Vmax] upper HSV value for the color of the frame
            
        Returns:
            img_rescaled: image with corrected perspective and resized according to frame
			M: Perspective transform
			width: Image width in pixel
			height: Image height in pixel
    """
	hsv, mask, field = colorFilter(img,lower_range, upper_range)
	
	field_gray = cv2.cvtColor(field, cv2.COLOR_BGR2GRAY)
	# noise
	filtered_img = cv2.bilateralFilter(field_gray,3,75,75)
	# binary
	binary_img = cv2.threshold(filtered_img, 20, 255, cv2.THRESH_BINARY_INV)[1]

	# find contours of MAP
	contours, _ = cv2.findContours(binary_img.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	approx_cont = []
	
	for cnt in contours:
		approx = cv2.approxPolyDP(cnt, 0.03*cv2.arcLength(cnt, True), True)
		area = cv2.contourArea(approx)
		# Criteria on the area to get only the external frame and not the obstacle or thymio if color filter did not work fine
		if (binary_img.shape[0]*binary_img.shape[1]/2 < area < binary_img.shape[0]*binary_img.shape[1]):
			approx_cont.append(approx.reshape(-1, 2))          
    
    # Correction of the perspective
	# First find the corners and figure out which one is which (top left / top right etc.)
	corners_tmp = approx_cont[0]
	corners_sorted = corners_tmp[corners_tmp[:, 1].argsort()]
	corners_top = corners_sorted[0:2,:]
	C_top_left = corners_top[corners_top[:, 0].argsort()][0,:]
	C_top_right = corners_top[corners_top[:, 0].argsort()][1,:]
	corners_bot = corners_sorted[2:,:]
	C_bot_left = corners_bot[corners_bot[:, 0].argsort()][0,:]
	C_bot_right = corners_bot[corners_bot[:, 0].argsort()][1,:]
	pts1 = np.float32([C_top_left, C_bot_left, C_bot_right, C_top_right])
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

	## Rescaling image (between width and height)
	img_rescaled = dst[0:height, 0:width]
	return img_rescaled, M, width, height

def mm2pixRatio(img):
	"""
	This function is used to know how many mm is one pixel

        Args:
			img: image 
    """
	global pix_per_mm
	# Conversion mm to pixel
	pix_width = img.shape[1]
	pix_per_mm = pix_width / MAP_WIDTH


def rescale_borders(img):
	"""
	This function is used to make the image slightly smaller and hence avoid to have small parts of the yellow frame visible

        Args:
			img: image of the rectified map
            
        Returns:
            map_rescaled: image of the map rescaled
    """
	map_rescaled = np.copy(img)
	map_rescaled = map_rescaled[10:img.shape[0]-10, 10:img.shape[1]-10]

	return map_rescaled
	

def findThymio(img, lower_range, upper_range):
	"""
	This function is used to find the position and angle of the thymio

        Args:
			img: image of the rescaled map
            lower range: [Hmin,Smin,Vmin] lower HSV value for the color of the thymio
			upper range: [Hmax,Smax,Vmax] upper HSV value for the color of the thymio
            
        Returns:
            img_rescaled: image with corrected perspective and resized according to frame
			pos: [posX, posY] position x and y of the thymio in pixel
			angle: angle of the thymio in rad
    """
	_, _, thymio = colorFilter(img,lower_range,upper_range)
	thymio_gray = cv2.cvtColor(thymio, cv2.COLOR_BGR2GRAY)
	thymio_gray = cv2.GaussianBlur(thymio_gray, (9,9), 5,5)
	# Set up the blob detector 
	params = cv2.SimpleBlobDetector_Params()
	params.filterByColor = False
	params.minThreshold = 100
	params.maxThreshold = 255
	params.filterByArea = True
	params.minArea = 70
	params.filterByCircularity = True
	params.minCircularity = 0.6
	detector = cv2.SimpleBlobDetector_create(params)
	
	# Detect blobs.
	keypoints = detector.detect(thymio_gray)
	# Make sure only two rounds were found. If not --> vision failed and make position and angle nan
	if len(keypoints) != 2:
		posX = np.nan
		posY = np.nan
		angle = np.nan
	else:
		pts = cv2.KeyPoint_convert(keypoints)
		if keypoints[0].size > keypoints[1].size:
			[posX, posY] = pts[0] # Big round coordinates
			[SRx, SRy] = pts[1] # Small round coordinates
			dx = SRx-posX
			dy = -(SRy-posY)
			angle = m.atan2(dy,dx)
		else:
			[posX, posY] = pts[1]
			[SRx, SRy] = pts[0]
			dx = SRx-posX
			dy = -(SRy-posY)
			angle = m.atan2(dy,dx)
	pos = [posX,posY]

	return pos, angle
	
def findGoal(img,lower_range,upper_range,TRESH,maxVal):
	"""
	This function is used to find the goal

        Args:
			img: image of the rescaled map
            lower range: [Hmin,Smin,Vmin] lower HSV value for the color of the goal 
			upper range: [Hmax,Smax,Vmax] upper HSV value for the color of the goal
			TRESH: Threshold gray value for thresholding function
			maxVal: Value assigned to pixel that have a value higher than the threshold
            
        Returns:
            GOAL: [x,y] coordinates of the goal
    """
	_, _, filt_img = colorFilter(img,lower_range,upper_range)
	goal_gray = cv2.cvtColor(filt_img, cv2.COLOR_BGR2GRAY)
	goal_binary = cv2.threshold(goal_gray, TRESH, maxVal, cv2.THRESH_BINARY_INV)[1]
	goal_binary = cv2.GaussianBlur(goal_binary, (5,5), 5,5)
	contours, _ = cv2.findContours(goal_binary.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	# Get the moments
	mu = [None]*len(contours)
	for i in range(len(contours)):
		mu[i] = cv2.moments(contours[i])
	# Get the mass centers
	mc = [None]*len(contours)
	for i in range(len(contours)):
		# add 1e-5 to avoid division by zero
		mc[i] = (mu[i]['m10'] / (mu[i]['m00'] + 1e-5), mu[i]['m01'] / (mu[i]['m00'] + 1e-5))

	# https://docs.opencv.org/3.4/d0/d49/tutorial_moments.html

	GOAL = [mc[1][0],mc[1][1]]
	return GOAL

def findObst(img,lower_range,upper_range,TRESH,maxVal):
	"""
	This function is used to find the obstacles

        Args:
			img: image of the rescaled map
            lower range: [Hmin,Smin,Vmin] lower HSV value for the color of the obstacles
			upper range: [Hmax,Smax,Vmax] upper HSV value for the color of the obstacles
			TRESH: Threshold gray value for thresholding function
			maxVal: Value assigned to pixel that have a value higher than the threshold
            
        Returns:
            approx_array_expended: List of n arrays containing the x,y coordinates of the edges of the n expanded obstacles
			approx_array_img: image of the expanded obstacles 
    """
	hsv, mask, filt_img = colorFilter(img,lower_range,upper_range)
	obst_gray = cv2.cvtColor(filt_img, cv2.COLOR_BGR2GRAY)
	_, binary_img = cv2.threshold(obst_gray, TRESH, maxVal, cv2.THRESH_BINARY)
	# find contours of obstacles
	contours, _ = cv2.findContours(binary_img.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	approx_array = []

	for cnt in contours:
			approx = cv2.approxPolyDP(cnt, 0.03 * cv2.arcLength(cnt, True), True)
			area = cv2.contourArea(approx)
			if 3000 < area < 10000:
				approx_array.append(approx.reshape(-1, 2))
	    

	exp_cnt = np.copy(approx_array)
	for i, cnt in enumerate(approx_array): # loop over obstacles
		for j, corner in enumerate(cnt): # loop over corners of obstacle
			e1 = cnt[(j-1)%cnt.shape[0]]-corner # find neighbors e1 and e2
			e2 = cnt[(j+1)%cnt.shape[0]]-corner
			bisector = (e1 / np.linalg.norm(e1) + e2 / np.linalg.norm(e2))
			exp_cnt[i][j] = corner - (pix_per_mm*THYMIO_WIDTH) * bisector / np.linalg.norm(bisector)

	exp_img = cv2.drawContours(binary_img.copy(), exp_cnt, -1, 255, thickness=cv2.FILLED)

	h, w = exp_img.shape[:2]
	mask = np.zeros((h+2, w+2), np.uint8)
	cv2.floodFill(exp_img, mask, (0,0), 123)
	exp_img = cv2.inRange(exp_img, 122, 124)
	exp_img = cv2.bitwise_not(exp_img)
	# https://stackoverflow.com/questions/70222415/opencv-how-to-merge-near-contours-to-get-the-one-big-outest-contour
	contours_exp, _ = cv2.findContours(exp_img.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	approx_array_exp = []
	
	for cnt in contours_exp:
		approx = cv2.approxPolyDP(cnt, 0.03 * cv2.arcLength(cnt, True), True)
		area = cv2.contourArea(approx)
		if 3000 < area < 80000:
			approx_array_exp.append(approx.reshape(-1, 2))


	exp_img_app = cv2.drawContours(exp_img.copy(), approx_array_exp, -1, 255, thickness=cv2.FILLED)

	return approx_array_exp, exp_img_app

############### Functions used in MAIN ############################################################

def map_init(img):
	"""
	This function is used to init the map by calling the function needed

        Args:
			img: image taken from the camera
            
        Returns:
            rescMap: Rectified and rescaled MAP
			M: Perspective transform (matrix)
			IMGwidth: width of the image (before rescaling) in pixel
			IMGheight: height of the image (before rescaling) in pixel
			mapsize: x,y size of the map in pixel
			goal_coords: x,y goal coordinates in pixel
			obst_coords: List of n arrays containing the x,y coordinates of the edges of the n expanded obstacles in pixel
			t_coords: x,y thymio coordinates in pixel
			t_angle: angle of thymio in rad
    """
	### Map rescaling and rectifying
	recMap, M, IMGwidth, IMGheight = mapTransform(img,YELLOW_L, YELLOW_H)
	mm2pixRatio(recMap)
	rescMap = rescale_borders(recMap)
	mapsize = [rescMap.shape[1],rescMap.shape[0]]
    
	### Find Goal
	goal_coords = findGoal(rescMap,RED_L,RED_H,20,255)
    
    ### Find Obstacles
	obst_coords, image = findObst(rescMap,BLUE_L,BLUE_H,20,10)
    
    ### Find Thymio
	t_coords, t_angle = findThymio(rescMap, GREEN_L, GREEN_H)
    
	return rescMap, M, IMGwidth, IMGheight, mapsize, goal_coords, obst_coords, t_coords, t_angle

def updateThymioPos(img, M, width, height):
	"""
	This function is used to find thymio on the map

        Args:
			img: image taken from the camera
			M: Perspective transform (matrix)
			width: width of the image (before rescaling) in pixel
			height: height of the image (before rescaling) in pixel
            
        Returns:
            rescMap: Rectified and rescaled MAP
			t_coords: x,y thymio coordinates in pixel
			t_angle: angle of thymio in rad
    """
    ### Map rescaling and rectifying
	dst = cv2.warpPerspective(img, M, (width, height))
	recMap = dst[0:height, 0:width]
	rescMap = rescale_borders(recMap)
	### Find Thymio
	t_coords, t_angle = findThymio(rescMap, GREEN_L, GREEN_H)

	return rescMap, t_coords, t_angle