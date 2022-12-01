import cv2
import numpy as np
import math as m
import time

# CONSTANT INIT
CAMERA = 2
k_sobel = 5

MAP_WIDTH = 1070 # mm
THYMIO_WIDTH = 110 # mm

RED_L = np.array([105,75,129],np.uint8)  # Set the Lower and higher range value of color for filter
RED_H = np.array([179,160,255],np.uint8)

GREEN_L = np.array([0, 60, 90],np.uint8)
GREEN_H = np.array([87, 255, 255],np.uint8)

BLUE_L = np.array([0, 79, 154],np.uint8)
BLUE_H = np.array([156, 255, 255],np.uint8)

YELLOW_L = np.array([0, 0, 158],np.uint8)
YELLOW_H = np.array([179, 240, 255],np.uint8)

half_thymio_pix = 23


def webcamShow(n):
    # Create a VideoCapture object and read from input file
    # If the input is the camera, pass 0 instead of the video file name
    cap = cv2.VideoCapture(n)

    # Check if camera opened successfully
    if (cap.isOpened()== False): 
        print("Error opening video stream or file")
    else:
        print("Everything fine")

    # Read until video is completed
    while(cap.isOpened()):
        # Capture frame-by-frame
        ret, frame = cap.read()
        if ret == True:

            # Display the resulting frame
            cv2.imshow('Frame',frame)

            # Press esc on keyboard to  exit
            if cv2.waitKey(1) == 27:
                break

        # Break the loop
        else: 
            break

        # When everything done, release the video capture object
    cap.release()

    # Closes all the frames
    cv2.destroyAllWindows()
    
    
def takePicture(n,img_name):
    cam = cv2.VideoCapture(n)
    check, frame = cam.read()
    #ROI = frame[0:1000,0:1000]

    if check == True :
        
        cv2.imwrite(img_name,frame)
    else :
        print("Error during the webcam opening")

    cam.release()
    return check

def loadImages(check,img_name):
    if check:
        mapImg = cv2.imread(img_name, cv2.IMREAD_COLOR)
        mapBw = cv2.imread(img_name, cv2.IMREAD_GRAYSCALE)
        eq = cv2.equalizeHist(mapBw)
        clahe = cv2.createCLAHE(clipLimit=0.5, tileGridSize=(8,8))
        eq = clahe.apply(mapBw)
        
        return mapImg, mapBw, eq

def globalFilter(eq):
    # Pre-processing the image through filtering and thresholding
    bilateral = cv2.bilateralFilter(eq,1,200,200)
    thresh = cv2.threshold(bilateral, 90, 255, cv2.THRESH_BINARY_INV)[1]

    # Applying morphological operators to the image
    kernelSquare13 = np.ones((13,13),np.float32)
    opened = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernelSquare13)
    
    return bilateral, thresh, opened

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
	tmp = cv2.threshold(filtered_img, 20, 255, cv2.THRESH_BINARY_INV)[1]

	# find contours of MAP
	contours, _ = cv2.findContours(tmp.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	approx_array = []
	circle = []
	for cnt in contours:
		approx = cv2.approxPolyDP(cnt, 0.03*cv2.arcLength(cnt, True), True)
		area = cv2.contourArea(approx)
		if (tmp.shape[0]*tmp.shape[1]/2 < area < tmp.shape[0]*tmp.shape[1]):
			approx_array.append(approx.reshape(-1, 2))          
    
    #rectifying
	pts1 = approx_array[0]
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

def findCorner(img, THRESH):
	# A modifier en utilisant filtre
	tmp = np.copy(img)
	tmp[tmp < THRESH] = 0
	tmp[tmp > THRESH] = 1
	#tmp[:50,:] = 0 # not needed if pic good
	rowN,colN = tmp.shape
	idx = np.where(tmp==1)
	corner_top_left = np.array([idx[1][0], idx[0][0]])
	corner_bottom_right = np.array([idx[1][-1], idx[0][-1]])

	done = False
	for j in range(0, m.floor(colN / 2)):  # iterating over columns
		for i in range(0, rowN):           # iterating over lines
			if tmp.item(i, j) == 1:
				corner_bottom_left = np.array([j, i])
				done = True
				break
	    # Exiting two for loops
		if done:
			break
	done = False
	for j in range(colN, m.floor(colN / 2), -1):  # iterating backwards over columns
		for i in range(0, m.floor(rowN/16)):  # iterating over rows !!!!!!! à changer
			if tmp.item(i, j - 1) == 1:
				corner_top_right = np.array([j - 1, i])
				done = True
				break
	    # Exiting two for loops, cf.: [SO2]
		if done:
			break
	pts1 = np.float32([corner_top_left, corner_top_right, corner_bottom_left, corner_bottom_right])
	return pts1

def rectify_map(img, pts1):
	# Target Points
	t1 = pts1[0][0]
	t2 = pts1[0][1]
	r1 = pts1[1][0]
	r2 = pts1[1][1]
	b1 = pts1[3][0]
	b2 = pts1[3][1]
	width = m.floor(m.sqrt((r1 - t1) ** 2 + (r2 - t2) ** 2))
	height = m.floor(m.sqrt((b1 - r1) ** 2 + (b2 - r2) ** 2))
	pts2 = np.float32([[0, 0], [width, 0], [0, height], [width, height]])
	M = cv2.getPerspectiveTransform(pts1, pts2)
	dst = cv2.warpPerspective(img, M, (width, height))

	## 3. Rescaling image (between width and height)
	rescaled = dst[0:height, 0:width]

	return rescaled 

def rescale_borders(img,mW,tW):
	# Conversion pixel to mm
	map_width = mW #mm
	pix_width = img.shape[1]
	pix_per_mm = pix_width / map_width
	thymio_width_mm = tW
	global half_thymio_pix
	half_thymio_pix = m.floor(thymio_width_mm*pix_per_mm/2)+1
	map_rescaled = np.copy(img)
	map_rescaled = map_rescaled[half_thymio_pix:img.shape[0]-half_thymio_pix,half_thymio_pix:img.shape[1]-half_thymio_pix]
	b_coords = [[0,0],[0,map_rescaled.shape[0]],[map_rescaled.shape[0],map_rescaled.shape[1]],[map_rescaled.shape[1],0]]
	return map_rescaled, b_coords
	

def findThymio(img, lower_range, upper_range, k_small, k_big, THRESH):
	hsv, mask, thymio = colorFilter(img,lower_range,upper_range)
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
	BR = [np.mean(idx[1]),np.mean(idx[0])]
	idx2 = np.where(tmp2==0)
	CR = [np.mean(idx2[1]),np.mean(idx2[0])]

	posX = np.mean([CR[0],BR[0]])
	posY = np.mean([CR[1],BR[1]])
	pos = [posX,posY]
	dx = CR[0]-BR[0]
	dy = -(CR[1]-BR[1])

	if (dx == 0) and (dy > 0):
		angle = np.pi / 2
	elif (dx == 0) and (dy < 0):
		angle = -np.pi / 2
	elif dx > 0:
		angle = np.arctan(dy/dx)
	else:
		angle = np.pi+np.arctan(dy/dx)
	return pos, angle

def findGoal(img,lower_range,upper_range,TRESH_L,TRESH_H):
	hsv, mask, filt_img = colorFilter(img,lower_range,upper_range)
	goal_gray = cv2.cvtColor(filt_img, cv2.COLOR_BGR2GRAY)
	tmp = cv2.threshold(goal_gray, TRESH_L, TRESH_H, cv2.THRESH_BINARY_INV)[1]
	tmp = cv2.GaussianBlur(tmp, (5,5), 5,5)
	idx = np.where(tmp==0)
	GOAL = [np.mean(idx[1]),np.mean(idx[0])]
	return GOAL

def findObst(img,lower_range,upper_range,TRESH_L,TRESH_H):
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
			exp_cnt[i][j] = corner - (half_thymio_pix + 15) * bisector / np.linalg.norm(bisector)

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

def map_init(img):
    start_init = time.time() # for time computing
    #### Map rescaling and rectifying
    recMap, M, width, height = mapTransform(img,YELLOW_L, YELLOW_H)
    rescMap, b_coords = rescale_borders(recMap,MAP_WIDTH,THYMIO_WIDTH)
    
    ### Find Goal
    goal_coords = findGoal(rescMap,RED_L,RED_H,20,255)
    
    ### Find Obstacles
    obst_coords, image = findObst(rescMap,BLUE_L,BLUE_H,20,10)
    
    ### Find Thymio
    t_coords, t_angle = findThymio(rescMap, GREEN_L, GREEN_H,3,17,20)
    
    end_init = time.time()
    t_init = end_init-start_init
    return rescMap, M, width, height, b_coords, goal_coords, obst_coords, t_coords, t_angle

def updateThymioPos(img):
    start_update = time.time()
    #### Map rescaling and rectifying
    recMap, M, width, height = mapTransform(img,YELLOW_L, YELLOW_H)
    rescMap, b_coords = rescale_borders(recMap,MAP_WIDTH,THYMIO_WIDTH)
    """
    dst = cv2.warpPerspective(img, M, (width, height))
    recMap = dst[0:height, 0:width]
    rescMap, b_coords = vision.rescale_borders(recMap,MAP_WIDTH,THYMIO_WIDTH)
    """
    ### Find Thymio
    t_coords, t_angle = findThymio(rescMap, GREEN_L, GREEN_H,3,17,20)
    
    end_update = time.time()
    t_update = end_update - start_update
    return rescMap, t_coords, t_angle

