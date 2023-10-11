#!/usr/bin/env python 3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float64MultiArray
import cv2
import numpy as np

# Load the calibration data
calibration_data = np.load(r'C:\Users\Girish\.vscode\programs\other stuff\calibration_data.npz')
mtx = calibration_data['mtx']
dist = calibration_data['dist']
class BallTrackerNode(Node):
    def __init__(self):
        super().__init__('ball_tracker_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Adjust the topic based on your camera setup
            self.image_callback,
            10)
        self.publisher_3d_coordinates = self.create_publisher(Float64MultiArray, 'ball_3d_coordinates', 10)
        self.cv_bridge = CvBridge()
        self.object_radius = 5  
        self.lower = np.array([0, 100, 100]) #HSV for red color 
        self.upper = np.array([10, 255, 255])

    def image_callback(self, msg):
        frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        undistorted_frame = cv2.undistort(frame, mtx, dist, None, mtx)
        hsv = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        x_3d, y_3d, z_3d = 0.0, 0.0, 0.0
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)


            center = (int(x), int(y))
            radius = int(radius)
            cv2.circle(undistorted_frame, center, radius, (0, 255, 0), 2)
            cv2.circle(undistorted_frame, center, 5, (0, 0, 255), -1)


            depth = (self.object_radius * mtx[0, 0]) / (2 * radius)
            x_3d, y_3d, z_3d = x, y, depth


        msg_3d_coordinates = Float64MultiArray()
        msg_3d_coordinates.data = [x_3d, y_3d, z_3d]
        self.publisher_3d_coordinates.publish(msg_3d_coordinates)


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
