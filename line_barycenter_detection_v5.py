import cv2
import numpy as np
import math
import os

def calculate_line_angle(image):
    """
    Calculates the angle between the vertical middle line of the image and
    the detected black line in the image.

    Args:
        image (numpy.ndarray): Input image (assumed to be in BGR format).
    
    Returns:
        angle (float): The angle in degrees. Negative for left turns, positive for right turns.
        processed_image (numpy.ndarray): Image with the detected line drawn.
        None, None: If no line is detected.
    """
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Threshold to detect the black line
    _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
    
    # Find contours of the black line
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        # If no contour is found, return None
        return None, image

    # Find the largest contour, assuming it's the black line
    largest_contour = max(contours, key=cv2.contourArea)
    
    # Fit a line to the contour using least squares
    [vx, vy, x, y] = cv2.fitLine(largest_contour, cv2.DIST_L2, 0, 0.01, 0.01)
    
    # Calculate the angle of the line relative to the vertical
    angle = math.atan2(vy, vx) * 180 / math.pi  # Convert from radians to degrees
    
    # Adjust angle so 0 degrees points upward, negative for left, positive for right
    #angle = -angle  # Flip direction (depending on convention, may not be needed)
    angle = angle - 90
    if angle <-90:
        angle = angle +180
    if angle > 90:
        angle = angle - 180

    # Draw the line on the image for visualization
    h, w = image.shape[:2]
    center = (int(x), int(y))  # Center point on the line
    line_length = max(h, w)  # Length of the line for visualization
    
    # Calculate two points to draw the line
    pt1 = (int(center[0] - line_length * vx), int(center[1] - line_length * vy))
    pt2 = (int(center[0] + line_length * vx), int(center[1] + line_length * vy))
    
    processed_image = image.copy()
    cv2.line(processed_image, pt1, pt2, (0, 255, 0), 2)  # Draw the fitted line in green
    cv2.line(processed_image, (w // 2, 0), (w // 2, h), (0, 0, 255), 2)  # Draw vertical center line in red
    
    return angle, processed_image

def process_images_in_directory(directory_path):
    """
    Processes all images in the given directory and calculates the line angle for each.
    Displays the images with the detected lines one at a time. The next image is shown
    only after the previous window is closed.

    Args:
        directory_path (str): Path to the directory containing images.
    """
    # Ensure the directory exists
    if not os.path.isdir(directory_path):
        print(f"Directory '{directory_path}' not found!")
        return

    # Get a list of all image files in the directory
    image_files = [f for f in os.listdir(directory_path) if f.lower().endswith(('.png', '.jpg', '.jpeg'))]

    if not image_files:
        print(f"No images found in the directory '{directory_path}'.")
        return

    print(f"Processing images in directory: {directory_path}")
    
    for image_file in image_files:
        image_path = os.path.join(directory_path, image_file)
        image = cv2.imread(image_path)

        if image is None:
            print(f"Failed to load image: {image_file}")
            continue

        angle, processed_image = calculate_line_angle(image)
        
        if angle is not None:
            print(f"{image_file}: Angle of the line = {angle:.2f} degrees")
        else:
            print(f"{image_file}: No line detected!")
        
        # Display the processed image with detected line
        cv2.imshow(f"Processed Image: {image_file}", processed_image)

        # Wait until the window is closed
        cv2.waitKey(0)
        cv2.destroyWindow(f"Processed Image: {image_file}")

    print("All images have been processed.")

# Example usage:
if __name__ == "__main__":
    directory = "Line_barycenter_test"  # Replace with your directory path
    process_images_in_directory(directory)
