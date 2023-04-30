import pickle
import time

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, PoseStamped

from python_controllers.src.helpers import (
    create_pose_stamped
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

poses = []
# Set our demo's initial pose
tag1_pose = pose_of_tag(nav, tags, "tag1")
poses.append(tag1_pose)

poses.append(pose_of_tag(nav, tags, "tag2"))

poses.append(pose_of_tag(nav, tags, "tag3"))

# initial_pose = PoseStamped()
# initial_pose.header.frame_id = 'map'
# initial_pose.header.stamp = nav.get_clock().now().to_msg()
# initial_pose.pose.position.x = -1.0
# initial_pose.pose.position.y = 0.0
# initial_pose.pose.position.z = 0.0
# initial_pose.pose.orientation.w = 1.0
# initial_pose.pose.orientation.z = 0.00

# initial_pose = PoseStamped()
# initial_pose.header.frame_id = 'map'
# initial_pose.header.stamp = nav.get_clock().now().to_msg()
# initial_pose.pose.position.x = 0.0
# initial_pose.pose.position.y = 0.0
# initial_pose.pose.position.z = 0.00
# initial_pose.pose.orientation.w = 0.0
# initial_pose.pose.orientation.z = 1.0
# poses.append(initial_pose)
#
# # Set our demo's initial pose
# initial_pose = PoseStamped()
# initial_pose.header.frame_id = 'map'
# initial_pose.header.stamp = nav.get_clock().now().to_msg()
# initial_pose.pose.position.x = 1.0
# initial_pose.pose.position.y = 0.0
# initial_pose.pose.position.z = 0.00
# initial_pose.pose.orientation.w = 1.0
# initial_pose.pose.orientation.z = 1.0
# poses.append(initial_pose)

nav.followWaypoints(poses)

while not nav.isTaskComplete():
    print('hi')

for pose in poses:
    print(pose)

result = nav.getResult()
print(result)
embed()