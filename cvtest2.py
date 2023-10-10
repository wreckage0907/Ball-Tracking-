import cv2
import numpy as np

# Load the calibration data
calibration_data = np.load(r'C:\Users\Girish\.vscode\programs\other stuff\calibration_data.npz')
mtx = calibration_data['mtx']
dist = calibration_data['dist']

# Known object dimensions (height, width, depth) in centimeters
object_radius = 5  # Assuming a spherical ball, adjust based on your ball's actual radius

# Initialize webcam
cap = cv2.VideoCapture(0)  # Use the appropriate camera index

# Lower and upper bounds for red color in HSV
# lower= np.array([0, 100, 100])
# upper= np.array([10, 255, 255])
lower = np.array([0, 0, 220])
upper = np.array([180, 30, 255])
# Loop to capture frames from the webcam
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Undistort the frame using camera calibration parameters
    undistorted_frame = cv2.undistort(frame, mtx, dist, None, mtx)

    # Convert the undistorted frame to HSV
    hsv = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2HSV)

    # Threshold the frame to get the red color
    mask = cv2.inRange(hsv, lower, upper)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw the contours on the original frame
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)

        # Draw the ball and center
        center = (int(x), int(y))
        radius = int(radius)
        cv2.circle(undistorted_frame, center, radius, (0, 255, 0), 2)
        cv2.circle(undistorted_frame, center, 5, (0, 0, 255), -1)

        # Estimate depth based on the known object radius and detected radius
        depth = (object_radius * mtx[0, 0]) / (2 * radius)
        x_3d, y_3d, z_3d = x, y, depth

        # Print the depth and 3D coordinates
        print("Depth (cm):", depth)
        print("3D Coordinates (x, y, z):", (x_3d, y_3d, z_3d))

    # Display the undistorted frame with detected ball
    cv2.imshow('Ball Detection', undistorted_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the OpenCV window
cap.release()
cv2.destroyAllWindows()
