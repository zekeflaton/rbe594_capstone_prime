import math
import os
import pickle
import time

from nav2_simple_commander.robot_navigator import BasicNavigator
from python_controllers.src.helpers import (
    write_line_to_file,
    quaternion_from_euler,
    create_pose_stamped,
    BatteryCharge,
    Pose
)

from python_controllers.src.motion_planners import (
    AStarPlanner
)
from python_controllers.src.tag_locations import tags


class Robot(object):

    def __init__(self, robot_name, charge_locations, orchestrator, max_x, max_y, initial_pose,
                 end_pose=None, motion_planner=None, metrics_file_path=None,
                 tags_filepath="../src/tags_file.pkl"):
        """

        :param str robot_name: name of the robot
        :param list(python_controllers.src.helpers.Pose): list of poses representing charging grids
        :param python_controllers.src.orchestrator.Orchestrator orchestrator: Orchestrator passes a copy of itself in
        :param int max_x: max x of the grid
        :param int max_y: max y of the grid
        :param python_controllers.src.helpers.Pose initial_pose: pose of start point
        :param python_controllers.src.helpers.Pose end_pose: pose of end point
        :param BaseMotionPlanner/None motion_planner: Optional override for a motion planner class
        :param str/None metrics_file_path: Optional file path to save metrics
        :param str tags_filepath: File path where april tag information is saved

        """
        self._path = []
        self.robot_name = robot_name
        self.max_x = max_x
        self.max_y = max_y
        self.current_pose = initial_pose
        self.charging_station = initial_pose
        self.shelf_pose = None
        self.end_pose = end_pose
        self.locked_cells = []
        self.orchestrator = orchestrator  # TODO remove this reference if possible
        self.motion_planner = AStarPlanner() if motion_planner is None else motion_planner
        self.observers = []
        self.battery_charge = BatteryCharge()
        self.charge_locations = charge_locations
        self.has_shelf = False
        self.metrics_file_path = metrics_file_path
        self._nav = BasicNavigator()
        self.tags = tags

    def plan_path(self):
        """
        Plan full path from start to end using A* and avoiding obstacles and store in _path class variable

        :return: bool: Did we plan successfully
        """
        start_time = time.time()
        parent_dict = {}
        if not self.end_pose:
            print("No end pose for robot {} so a path could not be planned".format(self.robot_name))
            return
        q, list_of_locations = self.motion_planner.initialize(self.current_pose.get_2d_pose(), self.end_pose.get_2d_pose())
        nodes_visited = 0

        while not q.empty():
            x, q, current_cost = self.motion_planner.get_next(q)
            nodes_visited += 1
            if x == self.end_pose.get_2d_pose():  # If we're at the goal, we're done
                self._path = self.backtrace(parent_dict)
                # pop the first pt because we're already there!
                self._path.pop(0)
                end_time = time.time()
                if self.metrics_file_path:
                    write_line_to_file(os.path.join(self.metrics_file_path, "compute_time_analysis.csv"),
                                       [self.motion_planner.__class__.__name__, str(end_time - start_time)])
                    path_length = len(self._path)
                    # +1 for each movement in x, y, or 90 degree rotation
                    best_path_length = abs(self.current_pose.x - self.end_pose.x) + abs(self.current_pose.y - self.end_pose.y) + abs(self.current_pose.theta - self.end_posetheta)/90
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

    def get_all_actions(self, pose_tuple, motion_planner):
        """

        :param tuple pose_tuple: Tuple of ints giving the (x,y) position as the source for new actions
        :param BaseMotionPlanner motion_planner: Motion planner to use for cost calculations
        :return: list(Tuple): list of tuples of (possible_action, cost_of_action)
        """
        all_possible_actions = []
        coordinates = []

        x_base, y_base, theta_base = pose_tuple
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

        for pose_tuple in coordinates:
            x, y, _ = pose_tuple
            # Ensure that the location has a valid tag and that the orchestrator has not locked the point
            if not self.orchestrator.is_pt_locked((x, y)) and (x, y, 0.011, 0, 0, 0) in self.tags.values():
                all_possible_actions.append((pose_tuple, motion_planner.cost(pose_tuple, self.end_pose.get_2d_pose())))
        
        return all_possible_actions

    def get_next_two_points(self):
        """
        Returns the next two path points
        :return: (Tuple(Tuple))
        """

        # If the robot is not at the end pose and we do not have a planned path, then plan one.
        if self.current_pose != self.end_pose and not self._path:
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


    @property
    def get_current_pose(self):
        """
        :return: Tuple: x and y coordinates of the current robot pose
        """
        return self.current_pose

    def backtrace(self, parent):
        path = [self.end_pose.get_2d_pose()]
        while path[-1] != self.current_pose.get_2d_pose():
            path.append(parent[path[-1]])
        path.reverse()
        return path

