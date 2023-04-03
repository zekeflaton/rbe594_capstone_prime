#!/usr/bin/env python
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import apriltag

class AprilTagDetectorClass():
	def __init__(self) -> None:
		self.bridge = CvBridge()
		self.options = apriltag.DetectorOptions(families="tag36h11")
		self.detector = apriltag.Detector(self.options)
		self.listener()

	def callback(self, data):
		try:
			img_string = np.fromstring(data.data, np.uint8)
			img_data = cv2.imdecode(img_string,cv2.IMREAD_COLOR)
			gray = cv2.cvtColor(img_data, cv2.COLOR_BGR2GRAY)
			results = self.detector.detect(gray)
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