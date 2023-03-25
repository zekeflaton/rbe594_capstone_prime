from nav2_simple_commander.robot_navigator import BasicNavigator, PoseStamped
import geometry_msgs
import rclpy

rclpy.init()
nav = BasicNavigator()

# Set our demo's initial pose
initial_pose = PoseStamped()
initial_pose.header.frame_id = 'map'
initial_pose.header.stamp = nav.get_clock().now().to_msg()
initial_pose.pose.position.x = 0.54
initial_pose.pose.position.y = -0.83
initial_pose.pose.position.z = 0.01
initial_pose.pose.orientation.w = 0.68
initial_pose.pose.orientation.z = -0.74
nav.goToPose(initial_pose)

while not nav.isTaskComplete():
    print('hi')

result = nav.getResult()
print(result)