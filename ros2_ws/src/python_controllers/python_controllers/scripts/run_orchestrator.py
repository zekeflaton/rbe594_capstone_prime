from argparse import ArgumentParser

import pickle
from IPython import embed

from python_controllers.src.orchestrator import (
    Orchestrator
)
from python_controllers.src.helpers import (
    pose_of_tag,
    load_shelves_dict,
    load_tags_dict
)

def main(num_robots, requests_to_make, DEBUG=False):
    """

    :param bool DEBUG: Are we in debug mode?  Start an embeded shell, otherwise just run
    :param int num_robots: number of robots to use
    :param int requests_to_make: number of job requests to make for the orchestrator
    """
    tags = load_tags_dict()
    shelves = load_shelves_dict()
    orchestrator = Orchestrator(
        shelves=shelves,
        size=(9, 7),
    )

    starts = []
    # TODO: Where should all of these robots started?  Can hardcode this if we have a good spot for it.
    for i in range(num_robots):
        robot_start_pose = pose_of_tag(tags, "tag{}".format(i))
        # Grab x (1st), y (2nd) and yaw/theta (3rd)
        starts.append([robot_start_pose[i] for i in (0, 1, 5)])


    goals = ["tag5"]
    # TODO: How to create these goals?  Need a way to know which tags have shelves and set the tags as goals.
    for i in range(num_robots):
        robot_start_pose = pose_of_tag("tag{}".format(i+50))
        # Grab x (1st), y (2nd) and yaw/theta (3rd)
        starts.append([robot_start_pose[i] for i in (0, 1, 5)])

    # create a queue of requests to be handled when a robot
    # is available
    while len(goals) > 0:
        orchestrator.make_request(goals.pop())

    # init each robot
    for count, start in enumerate(starts):
        goal = goals.pop()
        orchestrator.add_robot(count, start, goal)


    if DEBUG:
        embed(
            header="1:\n"
                   "orchestrator.make_request(goal_pose)\n"
                   "2:\n"
                   "while not orchestrator.is_done():\n"
                   "\torchestrator.move_all()\n"
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
