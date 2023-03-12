from PIL import Image
import numpy as np
from src.orchestrator import Orchestrator
from argparse import ArgumentParser
import pandas as pd
import random
from src.generate_warehouse_map import generate_warehouse_numpy_map

def run_analysis_sim(num_of_robots, shelves_to_grab):
    """
    :param int num_of_robots:  Indicates the number of robots to place
    """

    # load csv map
    ary_map = generate_warehouse_numpy_map(map_file='../src/warehouse.csv')
    obstacles = set()
    size = ary_map.shape
    robots = []
    charging_stations = []


    # identify the position of all the shelves
    # in the warehouse. Each robot will use a
    # shelf as its goal pose, so just take the first
    # n shelves encountered as goal poses
    for i in range(ary_map.shape[0]):
        for j in range(ary_map.shape[1]):
            px = sum(ary_map[i, j])
            if px == 0:
                obstacles.add((i, j))
            if px == 255:
                charging_stations.append((i,j,0))

    # add a robot to each charging station
    if num_of_robots > len(charging_stations):
        raise ValueError('Too many robots')
    else:
        starts = charging_stations[:num_of_robots]

    # create a list of all shelf locations
    # so that we can select at random
    shelf_list = list(obstacles)

    goals = [(x[0], x[1], 0) for x in shelf_list[:shelves_to_grab]]

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

    orchestrator = Orchestrator(obstacles, size)
    # register the deadlock observer
    orchestrator.subscribe_to_deadlock(deadlock_detected)
    painters = []
    colors = []
    for r in [0, 255]:
        for g in [0, 255]:
            for b in [0, 255]:
                colors.append((r,g,b))
    
    colors.remove((0,0,0))
    colors.remove((255,255,255))

    # init each robot
    for count, start in enumerate(starts):
        goal = goals.pop()
        robot = orchestrator.add_robot(count, start, goal)
        # register the move observer to paint
        # the robot path
        painters.append(Painter(colors[count % len(colors)]))
        robot.subscribe_to_movement(painters[-1].paint_move)
        robots.append(robot)

    # create a queue of requests to be handled when a robot
    # is available
    while len(goals) > 0:
        orchestrator.make_request(goals.pop())

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
    parser.add_argument("--num_robots", type=int, default=5)
    parser.add_argument("--requests_to_make", type=int, default=50)

    args = parser.parse_args()
    run_analysis_sim(args.num_robots, args.requests_to_make)
