from PIL import Image
import numpy as np
from obstacle_field_motion_planning import (
    Orchestrator,
    get_point_from_pose
)

# load png map
png_map = Image.open('../foo.png')
ary_map = np.array(png_map)
shelves = set()
# set the number of robots to place
num_of_robots = 5
robots = []
starts = []
goals = []
size = ary_map.shape

# identify the position of all the shelves
# in the warehouse. Each robot will use a 
# shelf as its goal pose, so just take the first
# n shelves encountered as goal poses
for i in range(ary_map.shape[0]):
    for j in range(ary_map.shape[1]):
        px = sum(ary_map[i, j])
        if px == 0:
            shelves.add((i, j))
            if len(goals) < num_of_robots:
                goals.append((i, j, 0))

# place the robots start poses
for i in range(0, 10, 2):
    starts.append((i, 0, 0))

# This class is just for painting robot
# paths on the png in different colors
class Painter:
    def __init__(self, color) -> None:
        self.color = color
    def paint_move(self, pt):
        ary_map[pt[0], pt[1]] = np.array(self.color)

# print a message when a dead lock is detected
def deadlock_detected(robot_name, pose):
    """

    :param str robot_name: name of the robot
    :param Tuple(int) pose: (x, y, theta) representing the robot pose
    :return:
    """
    print("Robot {} detected deadlock at {}, replanning...".format(robot_name, get_point_from_pose(pose)))


orchestrator = Orchestrator(shelves, size)
# register the deadlock observer
orchestrator.subscribe_to_deadlock(deadlock_detected)
painters = []
colors = [[255, 0, 0], [255, 255, 0],[0, 255, 0],[0, 0, 255],[255, 0, 255]]

# init each robot
for goal, start, id in zip(goals, starts, range(len(goals))):
    robot = orchestrator.add_robot(id, start, goal)
    # register the move observer to paint
    # the robot path
    painters.append(Painter(colors[id]))
    robot.subscribe_to_movement(painters[-1].paint_move)
    robots.append(robot)

# loop until all robots are done
while not orchestrator.is_done():
    orchestrator.move_all()

# save the result jpg
resultpng = Image.fromarray(ary_map)
resultpng.save('result.png')
result_resized = resultpng.resize((600, 600), Image.NEAREST)
result_resized.save('result_resized.png')
