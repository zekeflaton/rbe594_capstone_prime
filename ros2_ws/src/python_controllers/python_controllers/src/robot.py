import math
import os
import pickle
import time

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from python_controllers.src.helpers import (
    write_line_to_file,
    quaternion_from_euler,
    create_pose_stamped
)

from python_controllers.src.motion_planners import (
    AStarPlanner
)


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

    def __init__(self, robot_name, obstacles, charge_locations, orchestrator, max_x, max_y, initial_pose,
                 end_pose=(None, None, None), motion_planner=None, metrics_file_path=None,
                 tags_filepath="../src/tags_file.pkl"):
        """

        :param str robot_name: name of the robot
        :param set(Tuple) obstacles: set of tuples representing blocked grids
        :param set(Tuple) charge_locations: set of tuples representing charging grids
        :param src.orchestrator.Orchestrator orchestrator: Orchestrator passes a copy of itself in
        :param int max_x: max x of the grid
        :param int max_y: max y of the grid
        :param Tuple(float) initial_pose: (x, y, theta) of start point
        :param Tuple(float) end_pose: (x, y, theta) of end point
        :param BaseMotionPlanner/None motion_planner: Optional override for a motion planner class
        :param str/None metrics_file_path: Optional file path to save metrics
        :param str tags_filepath: File path where april tag information is saved

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
        self.metrics_file_path = metrics_file_path
        rclpy.init()
        self._nav = BasicNavigator()
        with open(tags_filepath, 'rb') as fp:
            self.tags = pickle.load(fp)
            print('tags dictionary saved loaded from file')

    def plan_path(self):
        """
        Plan full path from start to end using A* and avoiding obstacles and store in _path class variable

        :return: bool: Did we plan successfully
        """
        start_time = time.time()
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
                end_time = time.time()
                if self.metrics_file_path:
                    write_line_to_file(os.path.join(self.metrics_file_path, "compute_time_analysis.csv"),
                                       [self.motion_planner.__class__.__name__, str(end_time - start_time)])
                    path_length = len(self._path)
                    # +1 for each movement in x, y, or 90 degree rotation
                    best_path_length = abs(self.current_pose[0] - self.end_pose[0]) + abs(self.current_pose[1] - self.end_pose[1]) + abs(self.current_pose[2] - self.end_pose[2])/90
                    write_line_to_file(os.path.join(self.metrics_file_path, "path_efficiency_analysis.csv"),
                                       [self.motion_planner.__class__.__name__, str(best_path_length/path_length)])

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
        end_time = time.time()
        if self.metrics_file_path:
            write_line_to_file(os.path.join(self.metrics_file_path, "compute_time_analysis.csv"),
                               [self.motion_planner.__class__.__name__, str(end_time - start_time)])
        return False

    def get_all_actions(self, pose, motion_planner):
        """

        :param tuple pose: Tuple of ints giving the (x,y) position as the source for new actions
        :param BaseMotionPlanner motion_planner: Motion planner to use for cost calculations
        :return: list(Tuple): list of tuples of (possible_action, cost_of_action)
        """
        all_possible_actions = []
        coordinates = []

        x_base, y_base, theta_base = pose
        # test which axis the robot is aligned on
        # add forward and backward moves
        movement = 0.5
        if theta_base == 0:
            coordinates.append((x_base + movement, y_base, theta_base))
        elif theta_base == 90:
            coordinates.append((x_base, y_base + movement, theta_base))
        elif theta_base == 180:
            coordinates.append((x_base - movement, y_base, theta_base))
        elif theta_base == 270:
            coordinates.append((x_base, y_base - movement, theta_base))

        # correct turns in edge cases
        ccw_turn = theta_base - 90  # 90 degrees
        if ccw_turn < 0:
            ccw_turn = 270  # 270 degrees
        cw_turn = theta_base + 90  # 90 degrees
        if cw_turn > 270:  # 270 degrees
            cw_turn = 0

        # add possible turns
        coordinates.append((x_base, y_base, ccw_turn))
        coordinates.append((x_base, y_base, cw_turn))

        for x, y, theta in coordinates:
            # Ensure that the location has a valid tag and that the orchestrator has not locked the point
            if not self.orchestrator.is_pt_locked((x, y)) and (x, y, 0.011, 0, 0, 0) in self.tags.values():
                all_possible_actions.append(((x, y, theta), motion_planner.cost((x, y, theta), self.end_pose)))
        
        return all_possible_actions

    # def is_cord_inbounds(self, pose):
    #     """
    #     :param tuple pose: Tuple of ints giving the (x, y, theta) position as the source for new actions
    #     :return: bool: Is the coordinate inside the bounds of the map
    #     """
    #     x, y, theta = pose
    #     return 0 <= x < self.max_x and 0 <= y < self.max_y and 0 <= theta <= 360
    #
    # def is_cord_blocked_by_obstacle(self, pose):
    #     """
    #     :param tuple pose: Tuple of ints giving the (x, y, theta) position as the source for new actions
    #     :return: bool: Is the coordinate blocked by an obstacle
    #     """
    #     pt = get_point_from_pose(pose)
    #     return pt in self.obstacles

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

            waypoints = []
            # while self._path:
            next_path_pose = self._path.pop(0)
            print(next_path_pose)
            # Set our demo's initial pose
            # http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html
            next_pose_stamped = create_pose_stamped(
                nav=self._nav,
                x=next_path_pose[0] * 1.,  # convert to a float
                y=next_path_pose[1] * 1.,  # convert to a float
                z=0 * 1.,  # convert to a float
                roll=0 * 1.,  # convert to a float
                pitch=0 * 1.,  # convert to a float
                yaw=next_path_pose[2] * 1.,  # convert to a float
            )
            # http: // wiki.ros.org / tf2 / Tutorials / Quaternions
            # https: // answers.unity.com / questions / 147712 / what - is -affected - by - the - w - in -quaternionxyzw.html
            # https://www.programcreek.com/python/example/70252/geometry_msgs.msg.PoseStamped
            waypoints.append(next_pose_stamped)

            self._nav.followWaypoints(waypoints)
            while not self._nav.isTaskComplete():
                feedback = self._nav.getFeedback()
                print(feedback)
                # if feedback.navigation_duration > 600:
                #     self._nav.cancelTask()

            result = self._nav.getResult()
            print(result)

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
        """
        The robot is considered "done" and available for tasking if it has a shelf and is at the end/goal pose

        :return: bool is_done:
        """
        return self.current_pose == self.end_pose and self.has_shelf
    
    def set_new_endpoint(self, end_pose):
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

