from PIL import Image
from src.orchestrator import Orchestrator
from argparse import ArgumentParser
from src.generate_warehouse_map import generate_warehouse_numpy_map
from src.helpers import RobotPath, Counter


def run_analysis_sim(num_of_robots, shelves_to_grab, motion_planner=None, metrics_file_path=None):
    """

    :param int num_of_robots:  Indicates the number of robots to place
    :param shelves_to_grab:
    :param BaseMotionPlanner/None motion_planner: Optional override for a motion planner class
    :param str/None metrics_file_path: Optional file path to save metrics
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

    

    # track the number of deadlocks detected
    deadlock_counter = Counter()

    orchestrator = Orchestrator(
        shelves=obstacles,
        size=size,
        motion_planner=motion_planner,
        metrics_file_path=metrics_file_path
    )
    # register the deadlock observer
    orchestrator.subscribe_to_deadlock(lambda _: deadlock_counter.increment())
    
    robot_paths = []

    # init each robot
    for count, start in enumerate(starts):
        goal = goals.pop()
        robot = orchestrator.add_robot(count, start, goal)
        robot_paths.append(RobotPath(count))
        
        # register the move observer to track
        # the robot path        
        robot.subscribe_to_movement(robot_paths[-1].add_to_path)
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
    print("Total number of deadlocks: {} with {} robots".format(deadlock_counter.count, num_of_robots))
    return deadlock_counter.count, robot_paths


if __name__ == "__main__":
    parser = ArgumentParser(add_help=False)
    parser.add_argument("--num_robots", type=int, default=5)
    parser.add_argument("--requests_to_make", type=int, default=50)

    args = parser.parse_args()
    run_analysis_sim(args.num_robots, args.requests_to_make)
