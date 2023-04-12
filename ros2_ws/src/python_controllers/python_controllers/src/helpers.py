import math

import numpy as np
from nav2_simple_commander.robot_navigator import PoseStamped

# create a global list of possible
# RGB values
colors = []
for r in [0, 255]:
    for g in [0, 255]:
        for b in [0, 255]:
            colors.append((r,g,b))

colors.remove((0,0,0))
colors.remove((255,255,255))

# This class is just for painting robot
# paths on the png in different colors
class Painter:
    def __init__(self, color, map) -> None:
        self.color = color
        self.ary_map = map

    def paint_move(self, pt):
        self.ary_map[pt[0], pt[1]] = np.array(self.color)

# a class to store path info
# when subscribed to the robot obj
class RobotPath:
    def __init__(self, name) -> None:
        self.name = name
        self._path = []

    def add_to_path(self, pt):
        self._path.append(pt)

    @property
    def path(self):
        return self._path

# simple counter that subscribes
# to the deadlock event
class Counter:
    def __init__(self) -> None:
        self._count = 0

    def increment(self):
        self._count = self._count + 1

    @property
    def count(self):
        return self._count


def write_line_to_file(filepath, array, open_mode="a"):
    with open(filepath, open_mode) as f:
        f.write(",".join(array))
        f.write("\n")


def load_tags_dict(tags_filepath):
    """
    Load the pickled tags file into a dictionary object
    :param str tags_filepath: filepath for the pickled tags
    :return: dict tags: dictionary of tags.  key is tag name, value is tuple of (x,y,z, x_rot, y_rot, z_rot)
    """
    tags_filepath = "tags_file.pkl"
    with open(tags_filepath, 'rb') as fp:
        tags = pickle.load(fp)
        print('tags dictionary saved loaded from file')


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434

    :param float roll:
    :param float pitch:
    :param float yaw:
    :param list(float) quat: (x, y, z, w)
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q


def get_point_from_pose(pose):
    """
    We do not want to account for theta when address deadlocks, so we need to consider only the x and y coordinates

    :param Tuple(int) pose: (x, y, theta) pose of the robot
    :return: Tuple(int) pt: (x, y) location of the robot
    """
    x, y, _ = pose
    return x, y


def create_pose_stamped(nav, x, y, z, roll, pitch, yaw):
    """

    :param nav2_simple_commander.robot_navigator.BasicNavigator nav:
    :param float x:
    :param float y:
    :param float z:
    :param float roll:
    :param float pitch:
    :param float yaw:
    :return: nav2_simple_commander.robot_navigator.PoseStamped pose
    """
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = 'map'
    pose_stamped.header.stamp = nav.get_clock().now().to_msg()
    pose_stamped.pose.position.x = x
    pose_stamped.pose.position.y = y
    pose_stamped.pose.position.z = z
    quaternion = quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))
    pose_stamped.pose.orientation.x = quaternion[0]
    pose_stamped.pose.orientation.y = quaternion[1]
    pose_stamped.pose.orientation.z = quaternion[2]
    pose_stamped.pose.orientation.w = quaternion[3]
    return pose_stamped
