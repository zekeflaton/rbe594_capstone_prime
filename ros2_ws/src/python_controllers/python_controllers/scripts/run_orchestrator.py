from argparse import ArgumentParser

import pickle
from IPython import embed

from python_controllers.src.orchestrator import (
    Orchestrator
)
from python_controllers.src.helpers import (
    pose_of_tag,
    Pose
)
from python_controllers.src.shelf_locations import shelves
from python_controllers.src.tag_locations import tags


def main(num_robots, requests_to_make, DEBUG=False):
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
    )

    if num_robots > len(charge_locations):
        exit("Cannot start with more robots () than charge locations ()".format(num_robots, len(charge_locations)))
    starts = charge_locations[:num_robots]  # provide only as many starts as there are charge locations


    # init each robot
    for count, start in enumerate(starts):
        orchestrator.add_robot(count, start)

    # create a queue of requests to be handled when a robot
    # is available
    goals = []
    while len(goals) > 0:
        orchestrator.make_request(goals.pop(0))


    if DEBUG:
        orchestrator.robots[0].update_end_pose(Pose(0,0,0,0,0,0))
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
    else:
        # loop until all robots are done
        while not orchestrator.is_done():
            orchestrator.move_all()


if __name__ == "__main__":
    parser = ArgumentParser(add_help=False)
    parser.add_argument("--num_robots", type=int, default=5)
    parser.add_argument("--requests_to_make", type=int, default=50)
    parser.add_argument("--debug", type=bool, default=False)

    args = parser.parse_args()

    main(num_robots=args.num_robots, requests_to_make=args.requests_to_make, DEBUG=args.debug)
