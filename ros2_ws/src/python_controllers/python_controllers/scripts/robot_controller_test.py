import pickle
import math
from IPython import embed

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, PoseStamped

from python_controllers.src.helpers import (
    create_pose_stamped,
    pose_stamped_of_tag
)
from python_controllers.src.orchestrator import (
    Orchestrator
)


# rclpy.init()
# nav = BasicNavigator()


tags_filepath = "../src/tags_file.pkl"
with open(tags_filepath, 'rb') as fp:
    tags = pickle.load(fp)
    print('tags dictionary saved loaded from file')


orch = Orchestrator(
    shelves=[],
    size=(9, 7),
)

initial_pose = (0, -2, 90)
end_pose = (0, 2, 270)

orch.add_robot(
    robot_name="robot1",
    initial_pose=initial_pose,
    end_pose=end_pose
)

robot1 = orch.robots["robot1"]
nav = robot1._nav

embed()


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
