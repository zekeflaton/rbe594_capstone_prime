from argparse import ArgumentParser
import glob
from datetime import datetime
from IPython import embed
import os
from PIL import Image as im

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


def main(num_robots, requests_to_make, DEBUG=False, save_orch_output=False, sim=False):
    """

    :param int num_robots: number of robots to use
    :param int requests_to_make: number of job requests to make for the orchestrator
    :param bool DEBUG: Are we in debug mode?  Start an embeded shell, otherwise just run
    :param bool save_orch_output: save orch output to a set of images and a video file
    :param bool sim: are we controlling gazebo robots or simulating it with images
    """

    charge_locations = [
        pose_of_tag(tags, "tag151"),
        pose_of_tag(tags, "tag181"),
        pose_of_tag(tags, "tag211"),
        pose_of_tag(tags, "tag241"),
        pose_of_tag(tags, "tag271"),
    ]

    orch_output_filepath = "results//{}".format(datetime.now().isoformat())
    orchestrator = Orchestrator(
        shelves=shelves,
        charge_locations=charge_locations,
        size=(9, 7),
        debug=DEBUG,
        shelf_color=(0, 0, 0),  # black
        charge_location_color=(0, 255, 0),  # green
        sim=sim,
        orch_output_filepath=orch_output_filepath,
    )

    if num_robots > len(charge_locations):
        exit("Cannot start with more robots () than charge locations ()".format(num_robots, len(charge_locations)))
    starts = charge_locations[:num_robots]  # provide only as many starts as there are charge locations

    # init each robot
    colors = [
        (255, 0, 0),
        (0, 0, 255),
        (0, 255, 255),
        (255, 0, 255),
        (255, 255, 0)
    ]
    for count, start in enumerate(starts):
        orchestrator.add_robot(str(count), start, color=colors[count])

    # create a queue of requests to be handled when a robot
    # is available


    if DEBUG:
        print(orchestrator.robots)
        o = orchestrator
        r1 = o.robots["0"]
        if "1" in o.robots:
            r2 = o.robots["1"]
        if "2" in o.robots:
            r3 = o.robots["2"]
        if "3" in o.robots:
            r4 = o.robots["3"]
        if "4" in o.robots:
            r5 = o.robots["4"]
        shelf_tasks = ["A4", "D6", "C2", "B6", "D1"]
        # -4, -4 to 5, 3
        drop_off_locations = [(-4, -4), (-3, -3), (0, 0), (3, 3), (0, 1)]
        for i, r in orchestrator.robots.items():
            task = RobotTask(
                shelf_name=shelf_tasks[int(i)],
                drop_off_location=Pose(drop_off_locations[int(i)][0], drop_off_locations[int(i)][1], 0, 0, 0, 0),
                shelves=orchestrator.shelves
            )
            orchestrator.make_request(task)

        for i, r in orchestrator.robots.items():
            print("Robot name: ", r.robot_name)
            print("Robot color: ", r.color)

            r.plan_path()

        o.save_current_state_to_image()

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
                   "5:\n"
                   "while not orchestrator.is_done(): orchestrator.move_all()\n"
                   "6: \n"
                   "while not o.are_all_robots_at_home_without_task(): o.move_all()"
        )
    else:
        # loop until all robots are done
        while not orchestrator.is_done():
            orchestrator.move_all()


    # Create the frames
    frames = []
    imgs = sorted(glob.glob(os.path.join(orch_output_filepath, "*.png")))
    for i in imgs:
        new_frame = im.open(i)
        frames.append(new_frame)

    # Save into a GIF file that loops forever
    frames[0].save(os.path.join(orch_output_filepath, 'png_to_gif.gif'), format='GIF',
                   append_images=frames[1:],
                   save_all=True,
                   duration=300, loop=1)


if __name__ == "__main__":
    parser = ArgumentParser(add_help=False)
    parser.add_argument("--num_robots", type=int, default=5)
    parser.add_argument("--requests_to_make", type=int, default=50)
    parser.add_argument("--debug", type=bool, default=False)
    parser.add_argument("--save_orch_output", type=bool, default=False)
    parser.add_argument("--sim", type=bool, default=False)

    args = parser.parse_args()

    main(
        num_robots=args.num_robots,
        requests_to_make=args.requests_to_make,
        DEBUG=args.debug,
        save_orch_output=args.save_orch_output,
        sim=args.sim
    )


# for r_name, r in o.robots.items():
#     print("ROBOT NAME: ", r_name)
#     print(r.current_pose)
#     print(r.end_pose)
#     print(r._path)
#     print((r.current_pose == r.charge_locations[int(r_name)]) and not r._current_task)
