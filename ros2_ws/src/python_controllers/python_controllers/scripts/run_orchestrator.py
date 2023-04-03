import asyncio
from ros2_ws.src.python_controllers import Orchestrator
from argparse import ArgumentParser
from ros2_ws.src.python_controllers import generate_warehouse_numpy_map
from ros2_ws.src.python_controllers import RobotPath, Counter


async def main(warehouse_map_np):
    """

    :param numpy.ndarray warehouse_map_np: 3D image array.  2D provides floor layout and third D provides obstacle info
    :return:
    """

    obstacles = set()
    size = warehouse_map_np.shape
    robots = []
    charging_stations = []

    # identify the position of all the shelves
    # in the warehouse. Each robot will use a
    # shelf as its goal pose, so just take the first
    # n shelves encountered as goal poses
    for i in range(warehouse_map_np.shape[0]):
        for j in range(warehouse_map_np.shape[1]):
            px = sum(warehouse_map_np[i, j])
            if px == 0:
                obstacles.add((i, j))
            if px == 255:
                charging_stations.append((i, j, 0))

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
    # orchestrator.subscribe_to_deadlock(lambda _: deadlock_counter.increment())

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


if __name__ == "__main__":
    parser = ArgumentParser(add_help=False)
    parser.add_argument("--num_robots", type=int, default=5)
    parser.add_argument("--requests_to_make", type=int, default=50)

    args = parser.parse_args()

    # load csv map
    warehouse_map = generate_warehouse_numpy_map(map_file='../src/warehouse.csv')

    asyncio.create_task(main(warehouse_map_np=warehouse_map))
