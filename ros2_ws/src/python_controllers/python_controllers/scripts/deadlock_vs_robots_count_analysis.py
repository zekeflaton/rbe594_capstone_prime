from run_analysis_sim import run_analysis_sim
import pandas as pd
import matplotlib.pyplot as plt


def main():
    x = []
    y = []
    time = []
    for num_robots in range(1,30):
        deadlock_count, robot_paths = run_analysis_sim(num_of_robots=num_robots, shelves_to_grab=30)
        x.append(num_robots)
        y.append(deadlock_count)
        t = max([len(robot.path) for robot in robot_paths])
        time.append(t)
    
    data = pd.DataFrame({'num_robots': x, 'deadlock_count': y, 'time': time})
    # data = pd.read_csv('../results/deadlock_analysis.csv')
    fig, ax = plt.subplots()
    data.to_csv('../results/deadlock_analysis.csv')
    data.plot(
        xlabel='# of robots', 
        ylabel='# of collisions', 
        x='num_robots', 
        y='deadlock_count',
        legend=False,
        ax=ax)
    data.plot(
        xlabel='# of robots', 
        ylabel='# of cycles', 
        x='num_robots', 
        y='time',
        legend=False,
        ax=ax,
        secondary_y=True)
    fig.legend()
    fig.savefig('../results/deadlock_analysis.png')

if __name__ == "__main__":
    main()
