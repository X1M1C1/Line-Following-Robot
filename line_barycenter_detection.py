from __future__ import division
import cv2
import numpy as np
import math
# Input Image

def return_angle(image):


    # Resize image to 80x60
    #print("imagetextpour", image.shape)
    cv2.imshow("original_image",image)
    h, w = image.shape[:2]
    image=image[h//2:,w//20:(19*w//20)]
    cv2.imshow("croped image", image)


    # Convert to HSV color space
    blur = cv2.blur(image, (5, 5))
    ret, thresh1 = cv2.threshold(blur, 168, 255, cv2.THRESH_BINARY)
    hsv = cv2.cvtColor(thresh1, cv2.COLOR_RGB2HSV)
    #cv2.imshow("hsv", hsv)

    # Define range of white color in HSV
    lower_white = np.array([0, 0, 168])
    upper_white = np.array([172, 111, 255])

    # Threshold the HSV image
    mask = cv2.inRange(hsv, lower_white, upper_white)

    # Remove noise
    kernel_erode = np.ones((6, 6), np.uint8)
    eroded_mask = cv2.erode(mask, kernel_erode, iterations=1)

    kernel_dilate = np.ones((4, 4), np.uint8)
    dilated_mask = cv2.dilate(eroded_mask, kernel_dilate, iterations=1)

    # Find the different contours
    im2,contours, hierarchy = cv2.findContours(dilated_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
   

    # Sort by area (keep only the biggest one)
    #print(len(contours))
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]

    if len(contours) > 0:
        
        
        M = cv2.moments(contours[0])
        # Centroid
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        #print("Centroid of the biggest area: ({}, {})".format(cx, cy))

        # Given point coordinates
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        # Coordinates of the new origin (centered at base of the image)
        origin_x = w / 2
        origin_y = 0
        #print("origin coordinates", origin_x, origin_y)
        # Calculate the angle in radians
        if cx-origin_x >= 0:
            angle_rad =math.atan2(cy - origin_y, cx - origin_x)
            #print("the x sign is +")
        else :
            angle_rad =  math.atan2(cy - origin_y, cx - origin_x)
            #print("the x sign is -")
            

        
        # Convert angle to degrees
        angle_deg = math.degrees(angle_rad)
        cross_size = 10  
        cv2.line(im2, (cx - cross_size, cy), (cx+ cross_size, cy), (0,0,255),2)
        cv2.line(im2, (cx, cy - cross_size), (cx, cross_size+ cy), (0,0,255),2)
        cv2.imshow("im2", im2)
    else:
        print("No Centroid Found")
        angle_deg = 0
        
        
  

    return angle_deg-90


   
        
        
        
        
