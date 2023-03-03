from time import sleep
import random
import queue
import math
from motion_planners import (
    AStarPlanner
)


class Orchestrator(object):
    def __init__(self, num_robots):
        self.floor_map = set()
        self.robots = {}

    def add_robot(self, robot_name, initial_pose, end_pose):
        """

        :param int/str robot_name:
        :param initial_pose:
        :param end_pose:
        :return:
        """
        self.robots[robot_name] = RobotPathPlanner(self.floor_map, initial_pose, end_pose)

    def generate_map(self):
        pass

    def plan_it_all(self):
        # Plan all robot paths
        pass

    def plan_robot_path(self, robot):
        robot.plan_path()

    def advance_all_robots(self):
        # Check all robots next 2 locations and look for any collisions
        pass


class RobotPathPlanner(object):

    def __init__(self, floor_map, initial_pose, end_pose=(None, None)):
        """

        :param set(Tuple) floor_map: set of tuples representing blocked grids
        :param Tuple initial_pose: x,y of start point
        :param Tuple end_pose: x,y of end point
        """
        self._path = None
        self.current_pose = initial_pose
        self.end_pose = end_pose
        self.floor_map = floor_map

    def plan_path(self):
        """
        Plan full path from start to end using A* and avoiding obstacles and store in _path class variable

        :return: bool: Did I plan successfully
        """


    def get_next_points(self):
        # Return next 2 locations in path
        if self.path:
            return self.path[0], self.path[1]

    def move_robot(self):
        # Strip first element off path
        if self.path:
            self.current_pose = self.path.pop(0)

    def update_end_pose(self, end_pose):
        """

        :param Tuple end_pose: x,y of end point
        :return:
        """
        self.end_pose = end_pose

    @property
    def get_current_pose(self):
        return self.current_pose



def get_all_actions(self, coord, motion_planner):
    """
    Get a list of all possible locations we can move to from the given coordinate

    :param tuple coord: (x-cord, y-cord)
    :return: list[tuple] all_possible_actions: coordinates of all possible locations we could check
    """
    all_possible_actions = []
    coords = []
    coords.append((coord[0] + 1, coord[1]))
    coords.append((coord[0] - 1, coord[1]))
    coords.append((coord[0], coord[1] + 1))
    coords.append((coord[0], coord[1] - 1))
    # Counter is used to weight Dijikstras for fun.
    counter = 0
    for x, y in coords:
        counter += 1
        if self.is_cord_inbounds(x, y) and not self.is_cord_blocked_by_obstacle(x, y):
            all_possible_actions.append(((x, y), motion_planner.cost(x, y, self.x_g[0], self.x_g[1], counter)))
    return all_possible_actions

def run_script(self):
    self.x_g = (127, 127)
    self.create_obstacles()
    # self.reset_obstacles()
    with open("data.csv", "w") as f:
        f.write(
            ",".join(["coverage", "motion_planner", "nodes_visited", "len(path)"]))
        f.write("\n")
    for i in range(15):
        self.x_i = self.x_g
        self.x_g = (random.randint(0, 128), random.randint(0, 128))
        while self.is_cord_blocked_by_obstacle(self.x_g[0], self.x_g[1]):
            self.x_g = (random.randint(0, 128), random.randint(0, 128))
        # self.reset_obstacles()
        # obstacle_coverage = i*5+35
        # self.create_obstacles(coverage=obstacle_coverage)
        # print("obstacle coverage", obstacle_coverage)
        for j in range(len(self.planners)):
            success = False
            self.reset_planner()
            if j == 0:
                while not success:
                    success = self.plan_motion(j, True)
                    print("success", success, "coverage")
                    if success:
                        break
                    self.reset_planner()
                    # self.reset_obstacles()
                    # self.create_obstacles(coverage=obstacle_coverage)
            else:
                success = self.plan_motion(j, True)
                print("success", success)


def plan_motion(self, motion_planner_idx=None, run_silent=False, coverage=None):
    """
    Determine which motion planner was selected and use it

    :param motion_planner_idx:
    :param run_silent:
    :return:
    """
    if motion_planner_idx is None:
        motion_planner_idx = self.planner_options.curselection()[0]
    print("Running motion planning using: {}".format(self.planners[motion_planner_idx]))
    if motion_planner_idx == 0:
        return self.motion_plan(AStarPlanner(), run_silent)
    elif motion_planner_idx == 1:
        return self.motion_plan(BreadthFirstSearchPlanner(), run_silent)
    elif motion_planner_idx == 2:
        return self.motion_plan(DepthFirstSearchPlanner(), run_silent)
    elif motion_planner_idx == 3:
        return self.motion_plan(DijkstrasPlanner(), run_silent)
    elif motion_planner_idx == 4:
        return self.motion_plan(RandomPlanner(), run_silent)


def backtrace(self, parent):
    path = [self.x_g]
    while path[-1] != self.x_i:
        path.append(parent[path[-1]])
    path.reverse()
    return path


def color_planned_path(self, path):
    for coord in path:
        self.c.itemconfigure(self.get_grid_item(coord[0], coord[1]), fill="green")


def motion_plan(self, motion_planner, run_silent):
    """

    :param motion_planner:
    :param run_silent:
    :return: bool success: state whether we were able to chart a path
    """
    # List to hold search locations.  will contain a tuple of grid location (tuple of x and y) and "visited" boolean
    parent_dict = {}
    q, list_of_locations = motion_planner.initialize(self.x_i, self.x_g)
    nodes_visited = 0
    while not q.empty():
        x, q, current_cost = motion_planner.get_next(q)
        nodes_visited += 1
        self.c.itemconfigure(self.get_grid_item(x[0], x[1]), fill="blue")
        self.c.update()
        if x == self.x_g:  # If we're at the goal, we're done
            path = self.backtrace(parent_dict)
            self.color_planned_path(path)
            if not run_silent:
                messagebox.showinfo("SUCCESS", "Successfully planned path to goal location {}".format(self.x_g))
            print("motion_planner", motion_planner.__class__.__name__)
            print("nodes_visited", nodes_visited)
            print("path_length", len(path))
            if run_silent:
                with open("data.csv", "a") as f:
                    f.write(",".join([str(motion_planner.__class__.__name__), str(nodes_visited), str(len(path))]))
                    f.write("\n")
            return True

        # Iterate over all possible locations we could move to from our current location
        all_possible_actions = self.get_all_actions(x, motion_planner=motion_planner)
        if not all_possible_actions:
            continue
        for possible_action, cost_of_action in all_possible_actions:
            # Check if the new location is already visited
            if possible_action not in list_of_locations:
                parent_dict[possible_action] = x
                list_of_locations[possible_action] = True
                q = motion_planner.append_action(q, possible_action, cost=current_cost+cost_of_action)
    if not run_silent:
        messagebox.showinfo("Major Fail", "Failed to plan path to goal location {}".format(self.x_g))
    return False


if __name__ == '__main__':
    run_script()