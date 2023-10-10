import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class BallTrackerNode(Node):
    def __init__(self):
        super().__init__('ball_tracker_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Adjust the topic based on your camera setup
            self.image_callback,
            10)
        self.subscription  # Prevent unused variable warning

        self.cv_bridge = CvBridge()
        self.object_radius = 5  # Assuming a spherical ball, adjust based on your ball's actual radius
        self.lower = np.array([0, 0, 220])
        self.upper = np.array([180, 30, 255])

    def image_callback(self, msg):
        frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Undistort the frame using camera calibration parameters
        undistorted_frame = cv2.undistort(frame, mtx, dist, None, mtx)

        # Convert the undistorted frame to HSV
        hsv = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2HSV)

        # Threshold the frame to get the red color
        mask = cv2.inRange(hsv, self.lower, self.upper)

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
            depth = (self.object_radius * mtx[0, 0]) / (2 * radius)
            x_3d, y_3d, z_3d = x, y, depth

            # Print the depth and 3D coordinates
            print("Depth (cm):", depth)
            print("3D Coordinates (x, y, z):", (x_3d, y_3d, z_3d))

        # Display the undistorted frame with detected ball
        cv2.imshow('Ball Detection', undistorted_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    ball_tracker_node = BallTrackerNode()
    rclpy.spin(ball_tracker_node)
    ball_tracker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
