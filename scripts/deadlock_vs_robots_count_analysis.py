from run_analysis_sim import run_analysis_sim
import pandas as pd


def main():
    x = []
    y = []
    for num_robots in range(1,30):
        deadlock_count = run_analysis_sim(num_of_robots=num_robots, shelves_to_grab=30)
        x.append(num_robots)
        y.append(deadlock_count)
    
    data = pd.DataFrame({'num_robots': x, 'deadlock_count': y})
    data.to_csv('../results/deadlock_analysis.csv')
    fig = data.plot(
        xlabel='# of robots', 
        ylabel='# of collisions', 
        x='num_robots', 
        y='deadlock_count',
        legend=False)
    fig.figure.savefig('../results/deadlock_analysis.png')

if __name__ == "__main__":
    main()
