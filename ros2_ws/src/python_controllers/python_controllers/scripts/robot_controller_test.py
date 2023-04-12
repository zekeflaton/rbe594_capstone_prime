import pickle
import time
from IPython import embed

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, PoseStamped

from python_controllers.src.helpers import (
    create_pose_stamped
)
from python_controllers.src.orchestrator import (
    Orchestrator
)

rclpy.init()
nav = BasicNavigator()


tags_filepath = "../src/tags_file.pkl"
with open(tags_filepath, 'rb') as fp:
    tags = pickle.load(fp)
    print('tags dictionary saved loaded from file')


def pose_of_tag(navi, tagsi, tag_name):
    tag_data = tagsi[tag_name]
    print(tag_data)
    return create_pose_stamped(
        nav=navi,
        x=tag_data[0],
        y=tag_data[1],
        z=tag_data[2],
        roll=tag_data[3],
        pitch=tag_data[4],
        yaw=tag_data[5],
    )

orch = Orchestrator(
    shelves=[],
    size=[],
)

initial_pose = (0, 0, 0)
end_pose = (2, 2, 0)

orch.add_robot(
    robot_name="robot1",
    initial_pose=initial_pose,
    end_pose=end_pose
)


embed()
