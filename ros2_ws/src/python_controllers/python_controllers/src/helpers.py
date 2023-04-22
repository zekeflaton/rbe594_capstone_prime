import math
import pickle
import numpy as np
from nav2_simple_commander.robot_navigator import PoseStamped


class RobotTask(object):
    def __init__(self, shelf_name, drop_off_location, shelves):
        """

        :param str shelf_name: e.g. "D6".  Shelf name as listed in the world file
        :param Pose drop_off_location: Pose of drop off location
        :param dict shelves: keys are shelf names, values are Tuple(float) or the pose (x, y, z, roll, pitch, yaw)
        """
        self.shelf_name = shelf_name
        self.pick_up_location = Pose.pose_from_6d_tuple(shelves[shelf_name])
        self.drop_off_location = drop_off_location
        self.has_shelf = False
        self.complete = False

    def __repr__(self):
        return "python_controllers.src.helpers.RobotTask:\n\tshelf_name: {}\tpick_up_location: {}\n\tdrop_off_location: {}\n\thas_shelf: {}\n\tcomplete: {}\n".format(self.shelf_name, self.pick_up_location, self.drop_off_location, self.has_shelf, self.complete)


class Pose(object):

    def __init__(self, x, y, yaw, z=0, roll=0, pitch=0):
        """
        Can use this class as (x, y, theta) where yaw is theta os as a full 3D pose with translation and rotation.

        :param float x:
        :param float y:
        :param float z:
        :param float roll: in degrees
        :param float pitch: in degrees
        :param float yaw: in degrees (also used as theta)
        """
        # Multiple all by "1." to ensure they're floats
        self.x = x * 1.
        self.y = y * 1.
        self.z = z * 1.
        self.roll = roll * 1.
        self.pitch = pitch * 1.
        self.yaw = yaw * 1.

    def get_2d_pose(self):
        """
        Return 2d dimension pose (x, y, theta)
        :return: Tuple(float) pose:
        """
        return self.x, self.y, self.yaw

    def get_6d_pose(self):
        """
        Return 3d dimension pose (x, y, z, roll, pitch, yaw)
        :return: Tuple(float) pose:
        """
        return self.x, self.y, self.z, self.roll, self.pitch, self.yaw

    @property
    def theta(self):
        return self.yaw

    def get_xy(self):
        """
        returns x and y
        :return: Tuple(float):
        """
        return self.x, self.y

    @classmethod
    def manhattan_distance(cls, pose1, pose2):
        return abs(pose1.x - pose2.x) + abs(pose1.y - pose2.y)

    @classmethod
    def pose_from_6d_tuple(cls, tuple):
        x, y, z, roll, pitch, yaw = tuple
        return Pose(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)


    def __str__(self):
        return "Pose(x: {}\ty: {}\tz: {}\troll: {}\tpitch: {}\tyaw: {})\n".format(
            self.x, self.y, self.z, self.roll, self.pitch, self.yaw
        )

    def __repr__(self):
        return "Pose(x: {}\ty: {}\tz: {}\troll: {}\tpitch: {}\tyaw: {})\n".format(
            self.x, self.y, self.z, self.roll, self.pitch, self.yaw
        )

    def __eq__(self, other):
        if isinstance(other, Pose) and self.x == other.x and self.y == other.y and self.z == other.z and self.roll == other.roll and self.pitch == other.pitch and self.yaw == other.yaw:
            return True
        return False

    def __hash__(self):
        return hash(str(self))


class BatteryCharge(object):
    MIN_CHARGE = 0.0
    MAX_CHARGE = 100.0
    DRAIN_PER_MOVE = 0.5
    DRAIN_PER_CYCLE = 0.1
    CHARGE_PER_CYCLE = 20.0

    def __init__(self, initial_charge=None):
        """
        Initialize the battery level to the specified level or 0
        :param float/None initial_charge: amount of initial charge
        """
        if initial_charge is None:
            self.battery_charge = self.MAX_CHARGE
        else:
            self.battery_charge = self.bound_value_by_min_and_max(
                value=initial_charge,
                value_min=self.MIN_CHARGE,
                value_max=self.MAX_CHARGE
            )

    def drain_battery(self, drain_amount=None):
        """
        Drain battery charge by specific amount

        :param float drain_amount: amount to drain battery
        :return bool battery_out_of_charge: is the battery out of charge
        """
        battery_out_of_charge = False
        if not drain_amount:
            drain_amount = self.DRAIN_PER_MOVE
        if self.battery_charge - drain_amount < 0:
            battery_out_of_charge = True
        self.battery_charge = self.bound_value_by_min_and_max(self.battery_charge - drain_amount)
        return battery_out_of_charge

    def charge_battery(self, charge_amount=None):
        """
        Charge battery charge by specific amount

        :param float charge_amount: amount to charge battery
        """
        if not charge_amount:
            charge_amount = self.CHARGE_PER_CYCLE
        self.battery_charge = self.bound_value_by_min_and_max(self.battery_charge + charge_amount)

    def bound_value_by_min_and_max(self, value):
        """
        Ensure that a value is bounded by a min and max value.

        :param float value: the value to be bounded
        :return: float value: bounded between min and max
        """
        if value < self.MIN_CHARGE:
            return self.MIN_CHARGE
        elif value > self.MAX_CHARGE:
            return self.MAX_CHARGE
        return value


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


def load_tags_dict(tags_filepath="tags_file.pkl"):
    """
    Load the pickled tags file into a dictionary object

    :param str tags_filepath: filepath for the pickled tags
    :return: dict tags: dictionary of tags.  key is tag name, value is tuple of (x,y,z, x_rot, y_rot, z_rot)
    """
    with open(tags_filepath, 'rb') as fp:
        tags = pickle.load(fp)
        print('tags dictionary successfully loaded from file')
    return tags


def load_shelves_dict(shelves_filepath="../src/shelves_file.pkl"):
    """
    Load the pickled shelves file into a dictionary object

    :param str shelves_filepath: filepath for the pickled shelves dict
    :return: dict shelves: {shelf_name: (x, y, z, roll, pitch, yaw}
    """
    with open(shelves_filepath, 'rb') as fp:
        shelves = pickle.load(fp)
        print('shelves dictionary successfully loaded from file')
    return shelves


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
    if yaw == 270:
        yaw = -90
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    # Change ordering because it was w,x,y,z.  so q[3] used to be q[0]
    q[0] = cy * cp * sr - sy * sp * cr
    q[1] = sy * cp * sr + cy * sp * cr
    q[2] = sy * cp * cr - cy * sp * sr
    q[3] = cy * cp * cr + sy * sp * sr

    return q


# def get_point_from_pose(pose):
#     """
#     We do not want to account for theta when address deadlocks, so we need to consider only the x and y coordinates
#
#     :param Tuple(int) pose: (x, y, theta) pose of the robot
#     :return: Tuple(int) pt: (x, y) location of the robot
#     """
#     x, y, _ = pose
#     return x, y


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
    quaternion = quaternion_from_euler(roll, pitch, yaw)
    pose_stamped.pose.orientation.x = quaternion[0]
    pose_stamped.pose.orientation.y = quaternion[1]
    pose_stamped.pose.orientation.z = quaternion[2]
    pose_stamped.pose.orientation.w = quaternion[3]
    return pose_stamped


def pose_stamped_of_tag(nav, tags, tag_name):
    """
    Create a ROS PoseStamped message from a tag name

    :param nav2_simple_commander.robot_navigator.BasicNavigator nav: instance of BasicNavigator
    :param dict tags: Keys are tag names, values are tuples for the pose (x, y, z, roll, pitch, yaw)
    :param str tag_name: the name of the tag we want to get the pose for

    :return: nav2_simple_commander.robot_navigator.PoseStamped pose: ROS PoseStamped message
    """
    tag_pose = pose_of_tag(tags, tag_name)
    return create_pose_stamped(
        nav=nav,
        x=tag_pose.x,
        y=tag_pose.y,
        z=tag_pose.z,
        roll=tag_pose.roll,
        pitch=tag_pose.pitch,
        yaw=tag_pose.yaw,
    )


def pose_of_tag(tags, tag_name):
    """
    Create a ROS PoseStamped message from a tag name

    :param dict tags: Keys are tag names, values are tuples for the pose (x, y, z, roll, pitch, yaw)
    :param str tag_name: the name of the tag we want to get the pose for

    :return: RobotPose tag_data:
    """
    tag_data = tags[tag_name]
    tag_pose = Pose(
        x=tag_data[0],
        y=tag_data[1],
        z=tag_data[2],
        roll=tag_data[3],
        pitch=tag_data[4],
        yaw=tag_data[5],
    )
    return tag_pose
