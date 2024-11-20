import cv2
import numpy as np

def detect_edges(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    """
    lower_yellow = np.array([0, 180, 180])
    upper_yellow = np.array([179, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    #mask_yellow = cv2.Canny(mask_yellow, 200, 400)
    """
    lower = np.array([90, 0, 180])
    upper = np.array([170, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    count_black = np.sum(mask <= 10)
    w,h,_ = frame.shape
    
    if count_black > w*h*0.75:
        lower = np.array([0, 0, 170])
        upper = np.array([170, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
    
    return mask


def detect_intersection(frame):
    #find if there is an intersection between at least two lines in the cropped frame
    frame = cv2.resize(frame, (80, 60))

    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    #cv2.imshow('hsv', hsv)
    mask = detect_edges(frame)
    #cv2.imshow('mask', mask)
    

    # Noise reduction
    line_segments = cv2.HoughLinesP(mask, 1, np.pi / 180, 10, np.array([]), minLineLength=10, maxLineGap=4)
    #plt.imshow(dilated_mask)
    #print(len(line_segments))
    if line_segments is None:
        return False
    #Compute angle between all detected lines and return True if one is greater than 80°
    #range_i = np.random.randint(0, len(line_segments), 5)

    for i in range(len(line_segemnt)):
            for j in range(i+1, len(line_segment)):
                x1,y1,x2,y2 = line_segment[i][0]
                x3, y3, x4, y4 = line_segment[j][0]
                
                if x1 ==x2 or x3 == x4:
                    continue
                slope1= (y2-y1)/(x2-x1)
                slope2= (y4 - y3)/(x4- x3)
                
                if abs(slope1 - slope2) >0.05:
                    continue
                if abs(x1-x3)<10 or abs(x2-x4)<10:
                    angle= np.arctan2(np.abs(slope1-slope2),1+slope1*slope2)*180/np.pi
                    if angle > 80:
                        print("Intersection!")
                        return True
    return False
                
