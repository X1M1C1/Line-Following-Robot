import cv2
import numpy as np
import math
import os

def process_image(image):
    """
    Processes an input image to identify the largest black region, calculate its barycenter,
    determine the angle from vertical, and visualize the result.

    Args:
        image (numpy.ndarray): Input image (assumed to be in BGR format).

    Returns:
        angle_from_vertical (float): Angle in degrees between the line from the bottom center
                                     to the barycenter and the vertical line.
        image_with_lines (numpy.ndarray): Image with lines and angle visualization.
    """
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Threshold the image to isolate black regions
    _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

    # Find contours of the black regions
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        # If no contours are found, return None
        print("No valid black region detected!")
        return None, image

    # Find the largest contour
    largest_contour = max(contours, key=cv2.contourArea)

    # Calculate the moments of the largest contour
    moments = cv2.moments(largest_contour)
    if moments["m00"] == 0:  # Prevent division by zero
        print("Invalid region detected (zero area)!")
        return None, image

    # Calculate the barycenter (center of mass) of the black region
    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])
    barycenter = (cx, cy)

    # Prepare image for visualization
    image_with_lines = image.copy()

    # Draw the largest contour and the barycenter
    cv2.drawContours(image_with_lines, [largest_contour], -1, (0, 255, 0), 2)  # Contour in green
    cv2.circle(image_with_lines, barycenter, 5, (0, 0, 255), -1)  # Barycenter in red

    # Image dimensions
    height, width, _ = image.shape

    # Bottom center of the image
    bottom_center = (width // 2, height - 1)

    # Center of the image (for the vertical line)
    image_center = (width // 2, height // 2)

    # Draw the vertical line for reference
    cv2.line(image_with_lines, image_center, bottom_center, (255, 0, 0), 2)  # Vertical line in blue

    # Draw the line from the bottom center to the barycenter
    cv2.line(image_with_lines, bottom_center, barycenter, (0, 255, 255), 2)  # Line to barycenter in yellow

    # Calculate the angle between the two lines
    dx = barycenter[0] - bottom_center[0]
    dy = bottom_center[1] - barycenter[1]  # Inverted y-axis for proper angle calculation

    # Calculate the angle from vertical
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

        print(f"Processing {image_file}...")
        angle, result_image = process_image(image)

        if angle is not None:
            print(f"{image_file}: Angle from vertical = {angle:.2f}°")
        else:
            print(f"{image_file}: No valid black region detected!")

        # Display the result image
        cv2.imshow(f"Result - {image_file}", result_image)

        # Wait until the window is closed
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    print("All images have been processed.")

# Example usage:
if __name__ == "__main__":
    directory = "Line_barycenter_test"  # Replace with your directory path
    process_images_debug(directory)
