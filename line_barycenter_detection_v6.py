import cv2
import numpy as np
import math
import os

def hybrid_angle_detection(image):
    """
    Merges the line angle calculation approach with the barycenter approach.
    Uses the barycenter method when the intersection of the detected line
    with the bottom horizontal line lies outside the central third of the image.

    Args:
        image (numpy.ndarray): Input image (assumed to be in BGR format).

    Returns:
        angle (float): The angle in degrees. Negative for left turns, positive for right turns.
        processed_image (numpy.ndarray): Image with visualization.
    """
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Threshold to detect the black line
    _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
    
    # Find contours of the black line
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None, image  # No contours found

    # Find the largest contour
    largest_contour = max(contours, key=cv2.contourArea)
    
    # Fit a line to the contour
    [vx, vy, x, y] = cv2.fitLine(largest_contour, cv2.DIST_L2, 0, 0.01, 0.01)
    
    # Image dimensions
    h, w = image.shape[:2]
    center_third_left = w // 3
    center_third_right = 2 * (w // 3)

    # Calculate intersection with the bottom horizontal line
    if vy == 0:  # Avoid division by zero
        return None, image

    bottom_intersection_x = int(x - (y / vy) * vx)

    # Check if the intersection is in the central third
    if center_third_left <= bottom_intersection_x <= center_third_right:
        # Calculate the angle of the line relative to the vertical
        angle = math.atan2(vy, vx) * 180 / math.pi
        angle -= 90  # Adjust to make 0° vertical
        if angle < -90:
            angle += 180
        if angle > 90:
            angle -= 180

        # Draw the detected line and center vertical
        processed_image = image.copy()
        line_length = max(h, w)
        pt1 = (int(x - line_length * vx), int(y - line_length * vy))
        pt2 = (int(x + line_length * vx), int(y + line_length * vy))
        cv2.line(processed_image, pt1, pt2, (0, 255, 0), 2)  # Detected line in green
        cv2.line(processed_image, (w // 2, 0), (w // 2, h), (0, 0, 255), 2)  # Center vertical in red
    else:
        # Use barycenter approach
        moments = cv2.moments(largest_contour)
        if moments["m00"] == 0:
            return None, image  # Invalid contour

        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        bottom_center = (w // 2, h - 1)

        dx = cx - bottom_center[0]
        dy = bottom_center[1] - cy

        angle = math.degrees(math.atan2(dy, dx))
        angle_from_vertical = 90 - angle

        # Visualization
        processed_image = image.copy()
        cv2.drawContours(processed_image, [largest_contour], -1, (0, 255, 0), 2)
        cv2.circle(processed_image, (cx, cy), 5, (0, 0, 255), -1)  # Barycenter in red
        cv2.line(processed_image, bottom_center, (cx, cy), (0, 255, 255), 2)  # Line to barycenter in yellow
        cv2.line(processed_image, (w // 2, h // 2), bottom_center, (255, 0, 0), 2)  # Vertical in blue
        cv2.putText(processed_image, f"Angle: {angle_from_vertical:.2f}°", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        angle = angle_from_vertical

    return angle, processed_image

def process_images(directory_path):
    """
    Processes all images in the given directory and calculates the angle using the hybrid approach.

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

        angle, result_image = hybrid_angle_detection(image)

        if angle is not None:
            print(f"{image_file}: Angle = {angle:.2f}°")
        else:
            print(f"{image_file}: No valid line or region detected!")

        cv2.imshow(f"Processed - {image_file}", result_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    print("All images have been processed.")

# Example usage
if __name__ == "__main__":
    directory = "Line_barycenter_test"  # Replace with your directory path
    process_images(directory)
