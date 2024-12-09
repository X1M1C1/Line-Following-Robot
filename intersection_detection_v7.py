import cv2
import numpy as np
#import matplotlib.pyplot as plt
import os 

def process_image(img, blur_kernel_size=(105, 105)):#(50,05)
    # Read the image
    #img = cv2.imread(image_path, cv2.IMREAD_COLOR)

    h, w = img.shape[:2]
    img = img[h // 3 :, :]
    
    if img is None:
        print(f"Image at {image_path} could not be loaded.")
        return False

    # Apply Gaussian blur to reduce texture noise
    blurred_img = cv2.GaussianBlur(img, blur_kernel_size, 0)

    # Convert to grayscale
    gray = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2GRAY)
    
    # Apply adaptive thresholding to find black regions
    binary = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 751, 10
    )

    # Morphological opening to remove small noise
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)) #20/20
    binary_cleaned = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    
    # Detect contours
    contours, _ = cv2.findContours(binary_cleaned, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Create an output image to visualize
    output_img = blurred_img.copy()
    
    # Filter contours based on area
    min_area = 5000  # Set minimum area threshold
    contours = [c for c in contours if cv2.contourArea(c) > min_area]

    # Draw filtered contours
    cv2.drawContours(output_img, contours, -1, (0, 255, 0), 2)
    
    # Analyze contours for intersections
    intersection_detected = False
    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
        if len(approx) > 5:  # Intersection typically has multiple connecting lines
            intersection_detected = True
            cv2.drawContours(output_img, [contour], -1, (255, 0, 0), 2)
    
    # Visualize the steps
    # plt.figure(figsize=(12, 6))
    # plt.subplot(1, 3, 1)
    # plt.title("Original Image")
    # plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

    # plt.subplot(1, 3, 2)
    # plt.title("Blurred & Binary Image")
    # plt.imshow(binary, cmap='gray')

    # plt.subplot(1, 3, 3)
    # plt.title("Processed Image")
    # plt.imshow(cv2.cvtColor(output_img, cv2.COLOR_BGR2RGB))
    
    # plt.show()

    return intersection_detected, output_img


# Function to process all images in a directory
def process_images_in_directory(directory_path):
    # List all files in the directory
    all_files = os.listdir(directory_path)
    image_files = [f for f in all_files if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff'))]

    if not image_files:
        print(f"No image files found in directory: {directory_path}")
        return

    for image_file in image_files:
        image_path = os.path.join(directory_path, image_file)
        print(f"Processing {image_path}")
        result = process_image(image_path)
        print(f"Intersection Detected: {result}")


# Example Usage
#directory_path = "Generated_Images"  # Change to your directory path
#directory_path = "Test_Images"  # Change to your directory path
#process_images_in_directory(directory_path)
