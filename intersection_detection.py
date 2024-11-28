import cv2
import numpy as np
import matplotlib.pyplot as plt

def process_image(image_path):
    # Read the image
    img = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if img is None:
        print(f"Image at {image_path} could not be loaded.")
        return False

    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Threshold the image to find black regions
    _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
    
    # Detect contours
    contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Create an output image to visualize
    output_img = img.copy()
    cv2.drawContours(output_img, contours, -1, (0, 255, 0), 2)
    
    # Analyze contours for intersections
    intersection_detected = False
    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
        if len(approx) > 4:  # Intersection typically has multiple connecting lines
            intersection_detected = True
            cv2.drawContours(output_img, [contour], -1, (255, 0, 0), 2)
    
    # Visualize the steps
    plt.figure(figsize=(12, 6))
    plt.subplot(1, 3, 1)
    plt.title("Original Image")
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

    plt.subplot(1, 3, 2)
    plt.title("Binary Image")
    plt.imshow(binary, cmap='gray')

    plt.subplot(1, 3, 3)
    plt.title("Processed Image")
    plt.imshow(cv2.cvtColor(output_img, cv2.COLOR_BGR2RGB))
    
    plt.show()

    return intersection_detected

# Test with the provided images
image_paths = [
    "Generated_Images/test.png",
    "Generated_Images/corner.jpeg",
    "Generated_Images/straight_line.jpeg",
    "Generated_Images/intersection.jpeg",
    "Generated_Images/tjunct.jpeg",
]

for path in image_paths:
    print(f"Processing {path}")
    result = process_image(path)
    print(f"Intersection Detected: {result}")
