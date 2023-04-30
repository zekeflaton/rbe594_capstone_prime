import pickle
import math
from IPython import embed
import math
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, PoseStamped

from python_controllers.src.helpers import (
	create_pose_stamped,
	pose_stamped_of_tag
)
from python_controllers.src.orchestrator import (
	Orchestrator
)
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import String
import numpy as np
# rclpy.init()
# nav = BasicNavigator()
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import String

global_tag_pose_status =  False
global_tag_pose_data = None
global_tag_ID_data = None

def get_set_tag_pose(tagsi, tag_name):
	tag_data = tagsi[tag_name]
	print(tag_data)
	pose = np.array([[1,0,0,tag_data[0]],[0,1,0,tag_data[1]],[0,0,1,0],[0,0,0,1]])
	return pose

def get_robot_pose_from_tag(set_tag_pose, detected_tag_pose_wrt_robot):
	robot_pose = set_tag_pose @ np.linalg.inv(detected_tag_pose_wrt_robot)
	return robot_pose

class AprilDetector(Node):
	
	def __init__(self):
		super().__init__('AprilDetector_subscriber')
		self.subscription = self.create_subscription(
			AprilTagDetectionArray,
			'/detections',
			self.listener_callback,
			10)
		
		self.subscription  # prevent unused variable warning


	# def get_pose_from_homography(self, H):
	#     """
	#     Extracts pose (translation and rotation) from a homography matrix.

	#     Args:
	#         H (np.ndarray): 3x3 homography matrix.

	#     Returns:
	#         tuple: (translation, rotation)
	#             translation (np.ndarray): 3x1 translation vector.
	#             rotation (np.ndarray): 3x3 rotation matrix.
	#     """
	#     # Normalize the homography matrix
	#     H = H / H[2, 2]

	#     # Extract the rotation and translation from the homography matrix
	#     U, S, VT = np.linalg.svd(H[:2, :2])
	#     R = U @ VT
	#     T = H[:2, 2] / np.linalg.norm(H[:2, :2], axis=(0, 1))

	#     # Make sure the rotation matrix has a positive determinant
	#     if np.linalg.det(R) < 0:
	#         R = -R
	#         T = -T

	#     translation = T
	#     rotation = np.zeros((3, 3))
	#     rotation[:2, :2] = R
	#     rotation[2, 2] = 1
	#     tag_pose = np.zeros((4, 4))
	#     tag_pose[:3,:3] = rotation
	#     tag_pose[:2,3] = T
	#     print("T: ",T)
	#     tag_pose[2:,3] = np.array([0,1])
	#     return tag_pose
	def get_pose_from_homography(self,H):
		"""
		Computes the camera pose (i.e., rotation and translation) from a homography matrix H.

		Args:
		- H: 3x3 homography matrix
		- K: 3x3 camera intrinsic matrix

		Returns:
		- R: 3x3 rotation matrix
		- t: 3x1 translation vector
		"""
		K =np.array([[476.703,0,400.5],[0,476.703,400.5],[0,0,1]])
		h1, h2, h3 = H[:,0], H[:,1], H[:,2]
		K_inv = np.linalg.inv(K)

		# Compute the scaling factor
		lambda_inv = 1 / np.linalg.norm(np.dot(K_inv, h1))

		# Compute the rotation matrix
		r1 = lambda_inv * np.dot(K_inv, h1)
		r2 = lambda_inv * np.dot(K_inv, h2)
		r3 = np.cross(r1, r2)
		R = np.column_stack((r1, r2, r3))

		# Compute the translation vector
		t = lambda_inv * np.dot(K_inv, h3)
		print("rotation: ",R)
		print("translation: ",t)
		rotation = np.zeros((3, 3))
		rotation[:3, :3] = R
		tag_pose = np.zeros((4, 4))
		tag_pose[:3,:3] = rotation
		tag_pose[:3,3] = t
		tag_pose[3,3] = 1
		return tag_pose



	def listener_callback(self, msg):
		global global_tag_pose_data
		global global_tag_ID_data
		global global_tag_pose_status
		if len(msg.detections) >= 1:
			print("tag ID: ", msg.detections[0].id)
			global_tag_ID_data = msg.detections[0].id
			new_arrry = msg.detections[0].homography.reshape(3, 3)
			global_tag_pose_data = self.get_pose_from_homography(new_arrry)
			global_tag_pose_status = True

def calculate_yaw_from_rotation(R):
	"""
	Calculates the yaw angle from a 3x3 rotation matrix.

	Args:
	- R: 3x3 rotation matrix

	Returns:
	- yaw: yaw angle in radians
	"""

	# Extract the rotation angles from the rotation matrix
	roll, pitch, yaw = np.arctan2(R[2,1], R[2,2]), np.arctan2(-R[2,0], np.sqrt(R[2,1]**2 + R[2,2]**2)), np.arctan2(R[1,0], R[0,0])
	print("roll in rad: ",roll)
	print("pitch in rad: ",pitch)
	return yaw

def main(args=None):
	global global_tag_pose_status
	rclpy.init(args=args)
	AprilDetector_subscriber = AprilDetector()
	while rclpy.ok():
		if global_tag_pose_status == False:
			print("yes")
			rclpy.spin_once(AprilDetector_subscriber)
		else:
			break
	global_tag_pose_status = False        
	AprilDetector_subscriber.destroy_node()
	rclpy.shutdown()
	print("Tag ID: ",global_tag_ID_data)
	print("global_tag_pose_data: \n",global_tag_pose_data)
	tags_filepath = "../src/tags_file.pkl"
	with open(tags_filepath, 'rb') as fp:
		tags = pickle.load(fp)
		print('tags dictionary saved loaded from file')

	tag_ID = "tag"+str(global_tag_ID_data)
	tag_pose_from_world = get_set_tag_pose(tags,tag_ID)
	print("tag_pose_from_world: ",tag_pose_from_world)
	robot_location = get_robot_pose_from_tag(tag_pose_from_world,global_tag_pose_data )
	print("robot_location: ",robot_location)
	yaw = calculate_yaw_from_rotation(robot_location[:3,:3])
	print("yaw in rad: ",yaw)
	print("Robot Position X,Y in meters : \n",robot_location[:2,3])
	# orch = Orchestrator(
	# 	shelves=[],
	# 	size=(9, 7),
	# )
	# initial_pose = (0, -2, 90)
	# end_pose = (0, 2, 270)
	#
	# orch.add_robot(
	# 	robot_name="robot1",
	# 	initial_pose=initial_pose,
	# 	end_pose=end_pose
	# )

	# robot1 = orch.robots["robot1"]
	# nav = robot1._nav

	embed()

if __name__ == '__main__':
	main()
# from python_controllers.src.helpers import (
#     write_line_to_file,
#     quaternion_from_euler,
#     create_pose_stamped
# )
#
# pose_stamped = PoseStamped()
# pose_stamped.header.frame_id = 'map'
# pose_stamped.header.stamp = nav.get_clock().now().to_msg()
# pose_stamped.pose.position.x = -3.5
# pose_stamped.pose.position.y = -3.5
# pose_stamped.pose.position.z = 0.
# roll = 0.
# pitch = 0.
# yaw = 360.
# quaternion = quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))
# pose_stamped.pose.orientation.x = quaternion[0]
# pose_stamped.pose.orientation.y = quaternion[1]
# pose_stamped.pose.orientation.z = quaternion[2]
# pose_stamped.pose.orientation.w = quaternion[3]
#
# nav.goToPose(pose_stamped)

# to move the robot:  orch.move_all()
# or you can do:
# while not orch.is_done():
#   orch.move_all()
# to check the planned path: robot1._path
# orch.move_all(); robot1._path
