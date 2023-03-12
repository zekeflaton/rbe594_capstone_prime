import os
import pandas as pd

from run_analysis_sim import run_analysis_sim

from src.helpers import write_line_to_file
from src.motion_planners import (
    AStarPlanner,
    BreadthFirstSearchPlanner,
    DepthFirstSearchPlanner,
    RandomPlanner
)

def main():
    metrics_file_path = "../results"
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
    print(df.head())
    df = df[df.path_efficiency != 0]
    df["motion_planner"].replace("AStarPlanner", "A*", inplace=True)
    df["motion_planner"].replace("BreadthFirstSearchPlanner", "BFS", inplace=True)
    df["motion_planner"].replace("DepthFirstSearchPlanner", "DFS", inplace=True)
    df["motion_planner"].replace("RandomPlanner", "Random", inplace=True)

    print(df.head())
    mean = df.groupby(["motion_planner"]).mean()
    print(mean)

    fig = mean.plot(
        xlabel='Motion Planner',
        ylabel='Average Path Efficiency (%)',
        title="Average Path Efficiency vs Motion Planner",
        legend=False)

    fig.figure.savefig('../results/path_efficiency_analysis.png')


if __name__ == "__main__":
    main()
