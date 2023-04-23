from argparse import ArgumentParser
import os

import pickle
import pandas as pd
from IPython import embed

from python_controllers.src.orchestrator import (
    Orchestrator
)
from python_controllers.src.helpers import (
    pose_of_tag,
    Pose,
    RobotTask
)
from python_controllers.src.shelf_locations import shelves
from python_controllers.src.tag_locations import tags


def main(num_robots, requests_to_make, DEBUG=False, ACTUAL_PATH_DIR=None):
    """

    :param bool DEBUG: Are we in debug mode?  Start an embeded shell, otherwise just run
    :param int num_robots: number of robots to use
    :param int requests_to_make: number of job requests to make for the orchestrator
    """

    charge_locations = [
        pose_of_tag(tags, "tag151"),
        pose_of_tag(tags, "tag181"),
        pose_of_tag(tags, "tag211"),
        pose_of_tag(tags, "tag241"),
        pose_of_tag(tags, "tag271"),
    ]

    orchestrator = Orchestrator(
        shelves=shelves,
        charge_locations=charge_locations,
        size=(9, 7),
        debug=DEBUG,
        actual_path_dir=ACTUAL_PATH_DIR
    )

    if num_robots > len(charge_locations):
        exit("Cannot start with more robots () than charge locations ()".format(num_robots, len(charge_locations)))
    starts = charge_locations[:num_robots]  # provide only as many starts as there are charge locations


    # init each robot
    for count, start in enumerate(starts):
        orchestrator.add_robot(str(count), start)

    # create a queue of requests to be handled when a robot
    # is available


    if DEBUG:
        print(orchestrator.robots)
        o = orchestrator
        r = o.robots["0"]
        for i, robot in orchestrator.robots.items():
            task = RobotTask(
                shelf_name="D6",
                drop_off_location=Pose(int(i), int(i), 0, 0, 0, 0),
                shelves=orchestrator.shelves
            )
            orchestrator.make_request(task)
        embed(
            header="1:\n"
                   "orchestrator.make_request(goal_pose)\n"
                   "2:\n"
                   "while not orchestrator.is_done():\n"
                   "\torchestrator.move_all()\n"
                   "3:\n"
                   "orchestrator.move_all()\n"
                   "4:\n"
                   "orchestrator.robots[0].move_robot()\n"
        )
        file_path = os.path.join(orchestrator.actual_path_dir, r.robot_name, '.csv')
        df = pd.DataFrame(r.actual_path)
        df.to_csv(file_path)
    else:
        # loop until all robots are done
        while not orchestrator.is_done():
            orchestrator.move_all()


if __name__ == "__main__":
    parser = ArgumentParser(add_help=False)
    parser.add_argument("--num_robots", type=int, default=5)
    parser.add_argument("--requests_to_make", type=int, default=50)
    parser.add_argument("--debug", type=bool, default=False)
    parser.add_argument("--actual_path_dir", type=str, default=None)

    args = parser.parse_args()

    main(num_robots=args.num_robots, requests_to_make=args.requests_to_make, DEBUG=args.debug, ACTUAL_PATH_DIR=args.actual_path_dir)
