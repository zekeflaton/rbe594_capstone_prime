#!/usr/bin/env python
import re
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import apriltag
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class AprilTagDetectorClass():
	def __init__(self) -> None:
		self.bridge = CvBridge()
		self.options = apriltag.DetectorOptions(families="tag36h11")
		self.detector = apriltag.Detector(self.options)
		self.listener()
	def getPose(self, h_matrix):
		_,rotation_vec, translation_vec, _ = cv2.decomposeHomographyMat(h_matrix, np.eye(3))
		# R, _ = cv2.Rodrigues(rotation_vec)
		print(self.rotation_matrix_to_euler_angles(rotation_vec))
		# pose = np.hstack((R, translation_vec))
		# self.draw_pose(translation_vec, rotation_vec)
		print("Pose Matrix:\n", pose)
	def rotation_matrix_to_euler_angles(self, R):
		print("R: ",R)
		# Extract the rotation angles from the rotation matrix using the ZYX convention
		sy = np.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
		alpha = np.arctan2(R[2,1], R[2,2])
		beta = np.arctan2(-R[2,0], sy)
		gamma = np.arctan2(R[1,0], R[0,0])
		return np.array([alpha, beta, gamma])
	def draw_pose(self, position, rotation):
		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')

		# Draw arrow for the x-axis
		x_arrow = rotation @ np.array([1, 0, 0])
		ax.quiver(position[0], position[1], position[2], x_arrow[0], x_arrow[1], x_arrow[2], color='r')

		# Draw arrow for the y-axis
		y_arrow = rotation @ np.array([0, 1, 0])
		ax.quiver(position[0], position[1], position[2], y_arrow[0], y_arrow[1], y_arrow[2], color='g')

		# Draw arrow for the z-axis
		z_arrow = rotation @ np.array([0, 0, 1])
		ax.quiver(position[0], position[1], position[2], z_arrow[0], z_arrow[1], z_arrow[2], color='b')

		# Set axis limits and labels
		max_range = np.array([position, x_arrow, y_arrow, z_arrow]).max() * 1.1
		ax.set_xlim([-max_range, max_range])
		ax.set_ylim([-max_range, max_range])
		ax.set_zlim([-max_range, max_range])
		ax.set_xlabel('X')
		ax.set_ylabel('Y')
		ax.set_zlabel('Z')

		plt.show()

	def callback(self, data):
		try:
			img_string = np.fromstring(data.data, np.uint8)
			img_data = cv2.imdecode(img_string,cv2.IMREAD_COLOR)
			gray = cv2.cvtColor(img_data, cv2.COLOR_BGR2GRAY)
			results = self.detector.detect(gray)
			print("results: ",results)
			for r in results:
				# extract the bounding box (x, y)-coordinates for the AprilTag
				# and convert each of the (x, y)-coordinate pairs to integers
				(ptA, ptB, ptC, ptD) = r.corners
				ptB = (int(ptB[0]), int(ptB[1]))
				ptC = (int(ptC[0]), int(ptC[1]))
				ptD = (int(ptD[0]), int(ptD[1]))
				ptA = (int(ptA[0]), int(ptA[1]))
				# draw the bounding box of the AprilTag detection
				cv2.line(img_data, ptA, ptB, (0, 255, 0), 2)
				cv2.line(img_data, ptB, ptC, (0, 255, 0), 2)
				cv2.line(img_data, ptC, ptD, (0, 255, 0), 2)
				cv2.line(img_data, ptD, ptA, (0, 255, 0), 2)
				# draw the center (x, y)-coordinates of the AprilTag
				(cX, cY) = (int(r.center[0]), int(r.center[1]))
				cv2.circle(img_data, (cX, cY), 5, (0, 0, 255), -1)
				# draw the tag family on the image
				tagID = str(r.tag_id)
				cv2.putText(img_data, tagID, (ptA[0], ptA[1] - 15),
					cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
				print("[INFO] tag family: {}".format(tagID))
				self.getPose(r.homography)
		
		except CvBridgeError as e:
			print(e)
		# cv2.imshow("marker", tag)
		cv2.imshow("Image window", img_data)
		cv2.waitKey(1)

	def listener(self):

		rospy.init_node('listener', anonymous=True)
		rospy.Subscriber("/my_robot/camera1/image_raw/compressed", CompressedImage, self.callback)
		
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()

if __name__ == '__main__':
	april_tag_detector = AprilTagDetectorClass()