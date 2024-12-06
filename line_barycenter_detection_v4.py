import cv2
import numpy as np
import math
import os

def identify_black_barycenter(image):
    """
    Identifies the outlines of black regions in the image and calculates the barycenter of the largest black region.
    Draws debugging outlines and the barycenter for visualization.

    Args:
        image (numpy.ndarray): Input image (assumed to be in BGR format).
    
    Returns:
        barycenter (tuple): Coordinates of the barycenter (x, y) of the largest black region.
        outlined_image (numpy.ndarray): Image with outlines and barycenter drawn for debugging.
        None, None: If no valid black region is detected.
    """
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Threshold the image to isolate black regions
    _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

    # Find contours of the black regions
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        # If no contours are found, return None
        return None, image

    # Find the largest contour, assuming it's the primary black region of interest
    largest_contour = max(contours, key=cv2.contourArea)

    # Calculate the moments of the largest contour
    moments = cv2.moments(largest_contour)
    if moments["m00"] == 0:  # Prevent division by zero
        return None, image

    # Calculate the barycenter (center of mass) of the black region
    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])
    barycenter = (cx, cy)

    # Draw the contours and the barycenter on the image for debugging
    outlined_image = image.copy()
    cv2.drawContours(outlined_image, [largest_contour], -1, (0, 255, 0), 2)  # Draw contour in green
    cv2.circle(outlined_image, barycenter, 5, (0, 0, 255), -1)  # Draw barycenter in red

    return barycenter, outlined_image

def calculate_angle_and_draw(image, barycenter):
    """
    Calculates the angle between the line from the bottom-center of the image to the barycenter
    and the vertical line passing through the image center.

    Args:
        image (numpy.ndarray): Input image.
        barycenter (tuple): Coordinates of the barycenter.

    Returns:
        angle (float): Angle in degrees between the two lines.
        image_with_lines (numpy.ndarray): Image with lines and angle visualization.
    """
    height, width, _ = image.shape

    # Center of the bottom edge
    bottom_center = (width // 2, height - 1)

    # Center of the image (for the vertical line)
    image_center = (width // 2, height // 2)

    # Draw the vertical line for reference
    image_with_lines = image.copy()
    cv2.line(image_with_lines, image_center, bottom_center, (255, 0, 0), 2)  # Vertical line in blue

    # Draw the line from the bottom center to the barycenter
    cv2.line(image_with_lines, bottom_center, barycenter, (0, 255, 255), 2)  # Line to barycenter in yellow

    # Calculate the angle between the two lines
    dx = barycenter[0] - bottom_center[0]
    dy = bottom_center[1] - barycenter[1]  # Inverted y-axis for proper angle calculation

    # Vertical line direction is (0, -1), compute the angle with respect to this vector
    angle = math.degrees(math.atan2(dy, dx))
    angle_from_vertical = 90 - angle  # Convert to angle from the vertical line

    # Display the angle on the image
    angle_text = f"Angle: {angle_from_vertical:.2f}°"
    cv2.putText(image_with_lines, angle_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    return angle_from_vertical, image_with_lines

def process_images_debug(directory_path):
    """
    Processes all images in the given directory and identifies black barycenters.
    Displays the debugging output for each image.

    Args:
        directory_path (str): Path to the directory containing images.
    """
    if not os.path.isdir(directory_path):
        print(f"Directory '{directory_path}' not found!")
        return

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

        barycenter, outlined_image = identify_black_barycenter(image)

        if barycenter:
            print(f"{image_file}: Barycenter = {barycenter}")

            # Calculate the angle and visualize lines
            angle, result_image = calculate_angle_and_draw(outlined_image, barycenter)
            print(f"{image_file}: Angle from vertical = {angle:.2f}°")
        else:
            print(f"{image_file}: No valid black region detected!")
            result_image = outlined_image

        # Display the result image
        cv2.imshow("Result Image", result_image)

        # Wait until the window is closed
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    print("All images have been processed.")

# Example usage:
if __name__ == "__main__":
    directory = "Line_barycenter_test"  # Replace with your directory path
    process_images_debug(directory)
