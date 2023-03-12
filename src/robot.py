from .motion_planners import (
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


class BatteryCharge(object):
    MIN_CHARGE = 0.0
    MAX_CHARGE = 100.0

    def __init__(self, initial_charge=None):
        """
        Initialize the battery level to the specified level or 0
        :param float/None initial_charge: amount of initial charge
        """
        if initial_charge is None:
            self.battery_charge = self.MIN_CHARGE
        else:
            self.battery_charge = self.bound_value_by_min_and_max(
                value=initial_charge,
                value_min=self.MIN_CHARGE,
                value_max=self.MAX_CHARGE
            )

    def drain_battery(self, drain_amount):
        """
        Drain battery charge by specific amount

        :param float drain_amount: amount to drain battery
        """
        self.battery_charge = self.bound_value_by_min_and_max(self.battery_charge - drain_amount)

    def charge_battery(self, charge_amount):
        """
        Charge battery charge by specific amount

        :param float charge_amount: amount to charge battery
        """
        self.battery_charge = self.bound_value_by_min_and_max(self.battery_charge + charge_amount)

    def bound_value_by_min_and_max(self, value):
        """
        Ensure that a value is bounded by a min and max value.

        :param float value: the value to be bounded
        :return: float value: bounded between min and max
        """
        if value < self.MIN_CHARGE:
            return self.MIN_CHARGE
        elif value > self.MAX_CHARGE:
            return self.MAX_CHARGE
        return value


class Robot(object):

    def __init__(self, robot_name, obstacles, charge_locations, orchestrator, max_x, max_y, initial_pose, end_pose=(None, None, None),
                 motion_planner=None):
        """

        :param str robot_name: name of the robot
        :param set(Tuple) obstacles: set of tuples representing blocked grids
        :param set(Tuple) charge_locations: set of tuples representing charging grids
        :param src.orchestrator.Orchestrator orchestrator: Orchestrator passes a copy of itself in
        :param int max_x: max x of the grid
        :param int max_y: max y of the grid
        :param Tuple initial_pose: x,y of start point
        :param Tuple end_pose: x,y of end point
        :param BaseMotionPlanner motion_planner: Optional override for a motion planner class
        """
        self._path = []
        self.robot_name = robot_name
        self.obstacles = obstacles
        self.max_x = max_x
        self.max_y = max_y
        self.current_pose = initial_pose
        self.charging_station = initial_pose
        self.shelf_pose = end_pose
        self.end_pose = end_pose
        self.locked_cells = []
        self.orchestrator = orchestrator  # TODO remove this reference if possible
        self.motion_planner = AStarPlanner() if motion_planner is None else motion_planner
        self.observers = []
        self.battery_charge = BatteryCharge()
        self.charge_locations = charge_locations
        self.has_shelf = False

    def plan_path(self):
        """
        Plan full path from start to end using A* and avoiding obstacles and store in _path class variable

        :return: bool: Did we plan successfully
        """
        parent_dict = {}
        if None in self.end_pose:
            print("No end pose for robot {} so a path could not be planned".format(self.robot_name))
            return
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

        # if there is no path, its deadlocked
        return False

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
            if self.is_cord_inbounds((x, y, theta)) and (not self.orchestrator.is_pt_locked((x, y)) or (x, y) == (self.shelf_pose[0], self.shelf_pose[1])):
                all_possible_actions.append(((x, y, theta), motion_planner.cost((x, y, theta), self.end_pose)))
        
        if len(all_possible_actions) == 0:
            print('haaaa')
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
        if not self._path:
            self.plan_path()

        if len(self._path) > 1:
            return self._path[0], self._path[1]
        elif len(self._path) == 1:
            return self._path[0], None
        else:
            return None, None

    def move_robot(self):
        if self.current_pose in self.charge_locations and not self._path:
            # TODO change this hardcoded value to something based on research
            self.battery_charge.charge_battery(5)
        else:
            self.current_pose = self._path.pop(0)
            for observer in self.observers:
                observer.__call__(self.current_pose)
            # TODO remove this hardcoded drain amount with a value based on load, turning, movement, etc
            self.battery_charge.drain_battery(1.0)
        
        # mark that the robot has reached the shelf
        if self.current_pose == self.shelf_pose:
            self.has_shelf = True

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
        return self.current_pose == self.charging_station and self.has_shelf
    
    def reset(self, end_pose):
        self.end_pose = end_pose
        self.shelf_pose = end_pose
        self.has_shelf = False


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

