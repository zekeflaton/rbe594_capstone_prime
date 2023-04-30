import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import String
import numpy as np

class AprilDetector(Node):

    def __init__(self):
        super().__init__('AprilDetector_subscriber')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


    def get_pose_from_homography(self, H):
        """
        Extracts pose (translation and rotation) from a homography matrix.

        Args:
            H (np.ndarray): 3x3 homography matrix.

        Returns:
            tuple: (translation, rotation)
                translation (np.ndarray): 3x1 translation vector.
                rotation (np.ndarray): 3x3 rotation matrix.
        """
        # Normalize the homography matrix
        H = H / H[2, 2]

        # Extract the rotation and translation from the homography matrix
        U, S, VT = np.linalg.svd(H[:2, :2])
        R = U @ VT
        T = H[:2, 2] / np.linalg.norm(H[:2, :2], axis=(0, 1))

        # Make sure the rotation matrix has a positive determinant
        if np.linalg.det(R) < 0:
            R = -R
            T = -T

        translation = np.hstack([T, 1])
        rotation = np.zeros((3, 3))
        rotation[:2, :2] = R
        rotation[2, 2] = 1
        print("rotation: ",rotation)
        print("translation: ",translation)


    def listener_callback(self, msg):
        if len(msg.detections) >= 1:
            print("tag ID: ", msg.detections[0].id)
            new_arrry = msg.detections[0].homography.reshape(3, 3)
            self.get_pose_from_homography(new_arrry)
        # else:
        #     print("no detection")


def main(args=None):
    rclpy.init(args=args)
    AprilDetector_subscriber = AprilDetector()
    rclpy.spin(AprilDetector_subscriber)
    AprilDetector_subscriber.destroy_node()
    rclpy.shutdown()
    # rclpy.spin(AprilDetector_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # AprilDetector_subscriber.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()