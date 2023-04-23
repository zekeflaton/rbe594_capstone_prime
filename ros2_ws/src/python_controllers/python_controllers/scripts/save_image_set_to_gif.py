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


def main(save_dir):

    frames = []
    imgs = sorted(glob.glob(os.path.join(save_dir, "*.png")))
    print(imgs)
    imgs.remove("warehouse_map_00000.png")
    for i in imgs:
        new_frame = im.open(i)
        frames.append(new_frame)

    # Save into a GIF file that loops forever
    frames[0].save(os.path.join(save_dir, 'multi_robot_warehouse.gif'), format='GIF',
                   append_images=frames[1:],
                   save_all=True,
                   duration=300, loop=1)


if __name__ == "__main__":
    parser = ArgumentParser(add_help=False)
    parser.add_argument("--num_robots", type=int, default=5)

    # Each robot starts with a job, how many extras do we want to assign the system?
    parser.add_argument("--save_dir", type=str)

    args = parser.parse_args()

    main(
        save_dir=args.save_dir,
    )
