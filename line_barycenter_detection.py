import cv2
import numpy as np
import math
from picamera2 import Picamera2
# Input Image

def return_angle(image):
    # Crop the image
    h, w = image.shape[:2]
    image = image[h // 2 :, w // 20 : (19 * w // 20)]
    cv2.imshow("Cropped Image", image)

    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply inverted binary threshold
    ret, thresh1 = cv2.threshold(gray, 168, 255, cv2.THRESH_BINARY_INV)

    # Remove noise
    kernel_erode = np.ones((6, 6), np.uint8)
    eroded_mask = cv2.erode(thresh1, kernel_erode, iterations=1)

    kernel_dilate = np.ones((4, 4), np.uint8)
    dilated_mask = cv2.dilate(eroded_mask, kernel_dilate, iterations=1)

    #Save image for debugging
    im2 = dilated_mask.copy()
    
    # Find contours for black regions
    contours, hierarchy = cv2.findContours(
        dilated_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )

    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]

    if len(contours) > 0:
        
        
        M = cv2.moments(contours[0])
        # Centroid
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        #print("Centroid of the biggest area: ({}, {})".format(cx, cy))


        # Coordinates of the new origin (centered at base of the image)
        origin_x = w / 2
        origin_y = 0
        angle_rad =  math.atan2(cy - origin_y, cx - origin_x)
        angle_deg = math.degrees(angle_rad)
        cross_size = 10  
        cv2.line(im2, (cx - cross_size, cy), (cx+ cross_size, cy), (0,0,255),2)
        cv2.line(im2, (cx, cy - cross_size), (cx, cross_size+ cy), (0,0,255),2)
        cv2.imshow("im2", im2)
    else:
        print("No Centroid Found")
        angle_deg = 0
        
        
  

    return angle_deg-90


   
def main():
    picam2 = Picamera2()
    picam2.start()


    image_path = "Generated_Images/corner.jpeg"
    
    image = picam2.capture_array()
    
    if image is None:
        print(f"Error: Unable to load the image from path '{image_path}'")
        return
    angle = return_angle(image)
    
    print(f"Calculated Angle: {angle:.2f} degrees")
    
    # Wait for a key press to close the displayed images
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
        
        
        
