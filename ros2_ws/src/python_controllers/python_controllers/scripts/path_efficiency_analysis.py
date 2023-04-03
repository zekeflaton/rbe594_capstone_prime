import os
import pandas as pd

from run_analysis_sim import run_analysis_sim

from ros2_ws.src.python_controllers import write_line_to_file
from ros2_ws.src.python_controllers import (
    AStarPlanner,
    BreadthFirstSearchPlanner,
    DepthFirstSearchPlanner,
    RandomPlanner
)


def main(generate_data=False):
    """
    :param bool generate_data: Should the sim be run to generate data
    """
    metrics_file_path = "../../../../../results"

    if generate_data:
        write_line_to_file(os.path.join(metrics_file_path, "path_efficiency_analysis.csv"),
                           ["motion_planner", "path_efficiency"], "w")
        motion_planners = [
            AStarPlanner(),
            BreadthFirstSearchPlanner(),
            DepthFirstSearchPlanner(),
            RandomPlanner()
        ]
        for motion_planner in motion_planners:
            for num_robots in range(20, 30):
                run_analysis_sim(num_of_robots=num_robots, shelves_to_grab=30, motion_planner=motion_planner, metrics_file_path=metrics_file_path)

    df = pd.read_csv(os.path.join(metrics_file_path, "path_efficiency_analysis.csv"))
    df = df[df.path_efficiency != 0]
    df["motion_planner"].replace("AStarPlanner", "A*", inplace=True)
    df["motion_planner"].replace("BreadthFirstSearchPlanner", "BFS", inplace=True)
    df["motion_planner"].replace("DepthFirstSearchPlanner", "DFS", inplace=True)
    df["motion_planner"].replace("RandomPlanner", "Random", inplace=True)

    print(df.head())
    mean = df.groupby(["motion_planner"]).mean()
    print(mean)

    fig = mean.plot.bar(
        xlabel='Motion Planner',
        ylabel='Average Path Efficiency (%)',
        title="Average Path Efficiency vs Motion Planner",
        legend=False,
        rot=0
    )

    fig.figure.savefig('../results/path_efficiency_analysis.png')


if __name__ == "__main__":
    generate_data = False
    main(generate_data=generate_data)
