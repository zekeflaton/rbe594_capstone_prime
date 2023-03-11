from run_analysis_sim import run_analysis_sim


def main():
    with open("../results/deadlock_analysis.csv", "w") as f:
        f.write(",".join(["num_robots", "deadlock_count"]))
        f.write("\n")
    for num_robots in range(10):
        deadlock_count = run_analysis_sim(num_of_robots=num_robots)
        with open("../results/deadlock_analysis.csv", "a") as f:
            f.write(",".join([str(num_robots), str(deadlock_count)]))
            f.write("\n")


if __name__ == "__main__":
    main()
