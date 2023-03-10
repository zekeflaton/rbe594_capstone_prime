from src.motion_planners import (
    AStarPlanner
)

def get_point_from_pose(pose):
    """
    We do not want to account for theta when address deadlocks, so we need to consider only the x and y coordinates

    :param Tuple(int) pose: (x, y, theta) pose of the robot
    :return: Tuple(int) pt: (x, y) location of the robot
    """
    x, y, _ = pose
    return x, y


class RobotPathPlanner(object):

    def __init__(self, obstacles, orchestrator, max_x, max_y, initial_pose, end_pose=(None, None),
                 motion_planner=None):
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
                    q = self.motion_planner.append_action(q, possible_action, cost=current_cost + cost_of_action)

    def get_all_actions(self, pose, motion_planner):
        """

        :param tuple x: Tuple of ints giving the (x,y) position as the source for new actions
        :param BaseMotionPlanner motion_planner: Motion planner to use for cost calculations
        :return: list(Tuple): list of tuples of (possible_action, cost_of_action)
        """
        all_possible_actions = []
        coordinates = []

        x_base, y_base, theta_base = pose
        # test which axis the robot is aligned on
        # add forward and backward moves
        is_y_aligned = theta_base % 180 == 0
        if is_y_aligned:
            coordinates.append((x_base, y_base + 1, theta_base))
            coordinates.append((x_base, y_base - 1, theta_base))
        else:
            coordinates.append((x_base + 1, y_base, theta_base))
            coordinates.append((x_base - 1, y_base, theta_base))

        # correct turns in edge cases
        ccw_turn = theta_base - 90
        if ccw_turn < 0:
            ccw_turn = 270
        cw_turn = theta_base + 90
        if cw_turn > 270:
            cw_turn = 0

        # add possible turns
        coordinates.append((x_base, y_base, ccw_turn))
        coordinates.append((x_base, y_base, cw_turn))

        for x, y, theta in coordinates:
            if self.is_cord_inbounds((x, y, theta)) and (not self.orchestrator.is_pt_locked((x, y)) or (x, y) == (self.end_pose[0], self.end_pose[1])):
                all_possible_actions.append(((x, y, theta), motion_planner.cost((x, y, theta), self.end_pose)))
        return all_possible_actions

    def is_cord_inbounds(self, pose):
        """
        :param tuple pose: Tuple of ints giving the (x, y, theta) position as the source for new actions
        :return: bool: Is the coordinate inside the bounds of the map
        """
        x, y, theta = pose
        return 0 <= x < self.max_x and 0 <= y < self.max_y and 0 <= theta <= 360

    def is_cord_blocked_by_obstacle(self, pose):
        """
        :param tuple pose: Tuple of ints giving the (x, y, theta) position as the source for new actions
        :return: bool: Is the coordinate blocked by an obstacle
        """
        pt = get_point_from_pose(pose)
        return pt in self.obstacles

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

