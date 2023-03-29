from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

from IPython.terminal.embed import InteractiveShellEmbed

from argparse import ArgumentParser
from src.python_controllers.src.generate_warehouse_map import generate_warehouse_numpy_map


def main():
    rclpy.init()
    nav = BasicNavigator()
    # orchestrator = Orchestrator(
    #     shelves=[],
    #     size=(256, 256),
    # )
    #
    # orchestrator.add_robot("test", (0, 0, 0), (1, 1, 0))

    InteractiveShellEmbed(
        banner1="Orchestrator Console",
        banner2=(
            "## Usage:\n"
            "## \t orchestrator."
            "## \t orchestrator."
        )
    )


if __name__ == "__main__":
    parser = ArgumentParser(add_help=False)
    parser.add_argument("--num_robots", type=int, default=5)
    parser.add_argument("--requests_to_make", type=int, default=50)

    args = parser.parse_args()

    # load csv map
    warehouse_map = generate_warehouse_numpy_map(map_file='../src/warehouse.csv')

    main(warehouse_map_np=warehouse_map)