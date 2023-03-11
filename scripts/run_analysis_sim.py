from PIL import Image
import numpy as np
from src.orchestrator import Orchestrator
from argparse import ArgumentParser
import pandas as pd
import random

def run_analysis_sim(num_of_robots):
    """
    :param int num_of_robots:  Indicates the number of robots to place
    """

    # load csv map
    # csv_map = np.genfromtxt('src/warehouse.csv',delimiter=',', dtype=np.uint8)
    csv_map = pd.read_csv('../src/warehouse.csv', header=None).to_numpy()
    ary_map = np.ones((csv_map.shape[0], csv_map.shape[1], 3), dtype='uint8') * 255
    for i in range(csv_map.shape[0]):
        for j in range(csv_map.shape[1]):
            if csv_map[i,j] == 255:
                ary_map[i,j] = np.array([0,0,0])
    # ary_map = np.array(png_map)
    shelves = set()
    size = ary_map.shape
    robots = []

    # identify the position of all the shelves
    # in the warehouse. Each robot will use a
    # shelf as its goal pose, so just take the first
    # n shelves encountered as goal poses
    for i in range(ary_map.shape[0]):
        for j in range(ary_map.shape[1]):
            px = sum(ary_map[i, j])
            if px == 0:
                shelves.add((i, j))

    # create a list of all shelf locations
    # so that we can select at random
    shelf_list = list(shelves)
    _goals = set()
    _starts = set()
    while len(_goals) < num_of_robots:
        idx = random.randint(0, len(shelf_list)-1)
        _goals.add(idx)
        if len(_goals) == len(shelves):
            raise ValueError('Not enough shelfs to give all robots a goal')
    
    while len(_starts) < num_of_robots:
        xid = random.randint(0, size[0] - 1)
        yid = random.randint(0, size[1] - 1)
        _starts.add((xid, yid, 0))

    goals = [(shelf_list[idx][0], shelf_list[idx][1], 0) for idx in _goals]
    starts = list(_starts)

    # This class is just for painting robot
    # paths on the png in different colors
    class Painter:
        def __init__(self, color) -> None:
            self.color = color

        def paint_move(self, pt):
            ary_map[pt[0], pt[1]] = np.array(self.color)

    # print a message when a dead lock is detected
    def deadlock_detected(pt):
        print('Deadlock detected at ' + str(pt) + ', replanning...')

    orchestrator = Orchestrator(shelves, size)
    # register the deadlock observer
    orchestrator.subscribe_to_deadlock(deadlock_detected)
    painters = []
    colors = []
    for r in [0, 255]:
        for g in [0, 255]:
            for b in [0, 255]:
                colors.append((r,g,b))
    
    colors.remove((0,0,0))

    # init each robot
    for goal, start, count in zip(goals, starts, range(len(goals))):
        robot = orchestrator.add_robot(count, start, goal)
        # register the move observer to paint
        # the robot path
        painters.append(Painter(colors[count % len(colors)]))
        robot.subscribe_to_movement(painters[-1].paint_move)
        robots.append(robot)

    # loop until all robots are done
    while not orchestrator.is_done():
        orchestrator.move_all()

    # save the result jpg
    result_png = Image.fromarray(ary_map)
    result_png.save('../results/result.png')
    result_resized = result_png.resize((600, 600), Image.NEAREST)
    result_resized.save('../results/result_resized.png')
    print("Total number of deadlocks: {} with {} robots".format(orchestrator.deadlock_count, num_of_robots))
    return orchestrator.deadlock_count


if __name__ == "__main__":
    parser = ArgumentParser(add_help=False)
    parser.add_argument("--num_robots", type=int, default=50)

    args = parser.parse_args()
    run_analysis_sim(args.num_robots)
