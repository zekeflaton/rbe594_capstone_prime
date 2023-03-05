from time import sleep
import random
import queue
import math
from motion_planners import (
    AStarPlanner
)


class Orchestrator(object):
    def __init__(self, shelves, size):
        self.shelves = shelves
        self.size = size
        self.robots: dict[str, RobotPathPlanner] = {}
        self.locked = set()
        self.deadloc_observers = []

    def add_robot(self, robot_name, initial_pose, end_pose):
        """

        :param int/str robot_name:
        :param initial_pose:
        :param end_pose:
        :return:
        """
        self.robots[robot_name] = RobotPathPlanner(self.shelves, self, self.size[0], self.size[1], initial_pose, end_pose)
        self.robots[robot_name].plan_path()
        first, second = self.robots[robot_name].get_next_two_points()
        self.lock_cells(self.robots[robot_name], first, second)
        return self.robots[robot_name]

    
    def move_all(self):
        """
        Execute a move for all of the robots under the orchestrator
        control. If a collision is detected, replan the path.
        """
        robots_to_move = [self.robots[r] for r in self.robots if not self.robots[r].is_done()]
        for robot in robots_to_move:
            # unlock reserved pts
            for pt in robot.locked_cells:
                self.locked.remove(pt)
            robot.locked_cells.clear()

            # move the robot to the next pose
            robot.move_robot()
            # if arrived at goal, we're done!
            if robot.is_done():
                continue

            # identify the next two pts we need to
            # lock for this robot
            first, second = robot.get_next_two_points()

            # if either is already reserved for another robot
            # we need to replan the path
            if self.is_pt_locked(first) or self.is_pt_locked(second):
                for observer in self.deadloc_observers:
                    observer.__call__(robot.current_pose)
                robot.plan_path()
                first, second = robot.get_next_two_points()

            # once an available path has been found,
            # reserve the first and second pts for this
            # robot
            self.lock_cells(robot, first, second)
                
    def lock_cells(self, robot, first, second=None):
        """
        
        This method should only be called from within the orchestrator
        as a conveinience for locking pts
        """
        self.locked.add(first)
        robot.locked_cells.append(first)
        if second is not None:
            self.locked.add(second)
            robot.locked_cells.append(second)


    def is_done(self):
        '''Test if the current pose matches the goal pose'''
        alldone = [self.robots[r].is_done() for r in self.robots]
        return all(alldone)


    def is_pt_locked(self, pt: tuple[int, int]):
        '''Test if a pt is either a shelf or reserved for a robot'''
        if pt is None:
            return False
        else:
            return pt in self.shelves or pt in self.locked
        
    def subscribe_to_deadlock(self, func):
        '''Call the assigned func when a deadlock is detected.
        The func will recieve the current pose of the robot being
        replanned.'''
        self.deadloc_observers.append(func)

    def unsubscribe_to_deadlock(self, func):
        self.deadloc_observers.remove(func)


class RobotPathPlanner(object):

    def __init__(self, obstacles, orchestrator: Orchestrator, max_x, max_y, initial_pose, end_pose=(None, None), motion_planner=None):
        """

        :param set(Tuple) obstacles: set of tuples representing blocked grids
        :param Orchestrator orchestrator: Orchestrator passes a copy of itself in
        :param int max_x: max x of the grid
        :param int max_y: max y of the grid
        :param Tuple initial_pose: x,y of start point
        :param Tuple end_pose: x,y of end point
        :param BaseMotionPlanner motion_planner: Optional override for a motion planner class
        """
        self._path = None
        self.obstacles = obstacles
        self.max_x = max_x
        self.max_y = max_y
        self.current_pose = initial_pose
        self.end_pose = end_pose
        self.locked_cells = []
        self.orchestrator = orchestrator  # TODO remove this reference if possible
        self.motion_planner = AStarPlanner() if motion_planner is None else motion_planner
        self.observers = []

    def plan_path(self):
        """
        Plan full path from start to end using A* and avoiding obstacles and store in _path class variable

        :return: bool: Did we plan successfully
        """
        parent_dict = {}
        q, list_of_locations = self.motion_planner.initialize(self.current_pose, self.end_pose)
        nodes_visited = 0
        while not q.empty():
            x, q, current_cost = self.motion_planner.get_next(q)
            nodes_visited += 1
            if x == self.end_pose:  # If we're at the goal, we're done
                self._path = self.backtrace(parent_dict)
                # pop the first pt because we're already there!
                self._path.pop(0)
                return True

            # Iterate over all possible locations we could move to from our current location
            all_possible_actions = self.get_all_actions(x, motion_planner=self.motion_planner)
            if not all_possible_actions:
                continue
            for possible_action, cost_of_action in all_possible_actions:
                # Check if the new location is already visited
                if possible_action not in list_of_locations:
                    parent_dict[possible_action] = x
                    list_of_locations[possible_action] = True
                    q = self.motion_planner.append_action(q, possible_action, cost=current_cost+cost_of_action)

    def get_all_actions(self, x, motion_planner):
        """

        :param tuple x: Tuple of ints giving the (x,y) position as the source for new actions
        :param BaseMotionPlanner motion_planner: Motion planner to use for cost calculations
        :return: list(Tuple): list of tuples of (possible_action, cost_of_action)
        """
        all_possible_actions = []
        coordinates = [
            (x[0] + 1, x[1]),
            (x[0] - 1, x[1]),
            (x[0], x[1] + 1),
            (x[0], x[1] - 1),
        ]
        for x, y in coordinates:
            if self.is_cord_inbounds(x, y) and (not self.orchestrator.is_pt_locked((x, y)) or (x,y) == self.end_pose):
                all_possible_actions.append(((x, y), motion_planner.cost(x, y, self.end_pose[0], self.end_pose[1])))
        return all_possible_actions

    def is_cord_inbounds(self, x, y):
        """

        :param int x: x position
        :param int y: y position
        :return: bool: Is the coordinate inside the bounds of the map
        """
        return 0 <= x < self.max_x and 0 <= y < self.max_y

    def is_cord_blocked_by_obstacle(self, x, y):
        """

        :param int x: x position
        :param int y: y position
        :return: bool: Is the coordinate blocked by an obstacle
        """
        return (x, y) in self.obstacles

    def get_next_two_points(self):
        """
        Returns the next two path points
        :return: (Tuple(Tuple))
        """
        if self._path is None:
            self.plan_path()

        if len(self._path) > 1:
            return self._path[0], self._path[1]
        elif len(self._path) == 1:
            return self._path[0], None
        else:
            print("We should never get here")
            return None

    def move_robot(self):
        self.current_pose = self._path.pop(0)
        for observer in self.observers:
            observer.__call__(self.current_pose)

    def subscribe_to_movement(self, func):
        self.observers.append(func)

    def unsubscribe_from_movement(self, func):
        self.observers.remove(func)

    def update_end_pose(self, end_pose):
        """
        Update the robots end pose with a new goal

        :param Tuple end_pose: x,y of end point
        """
        self.end_pose = end_pose

    def is_done(self):
        return self.current_pose == self.end_pose

    @property
    def get_current_pose(self):
        """
        :return: Tuple: x and y coordinates of the current robot pose
        """
        return self.current_pose

    def backtrace(self, parent):
        path = [self.end_pose]
        while path[-1] != self.current_pose:
            path.append(parent[path[-1]])
        path.reverse()
        return path

