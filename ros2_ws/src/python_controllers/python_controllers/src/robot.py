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
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class JointPistonNode(Node):
    def __init__(self):
        super().__init__("piston_vel_cont")
        self.piston_joint_publisher_ = self.create_publisher(
            Float64MultiArray, "piston_vel_cont/commands", 10)

    def publish_piston_up(self):
        msg = Float64MultiArray()
        msg.data = [0.2]
        self.piston_joint_publisher_.publish(msg)

    def publish_piston_down(self):
        msg = Float64MultiArray()
        msg.data = [0.0]
        self.piston_joint_publisher_.publish(msg)


class Robot(object):

    def __init__(self, robot_name, charge_locations, orchestrator, max_x, max_y, initial_pose,
                 end_pose=None, motion_planner=None, metrics_file_path=None,
                 tags_filepath="../src/tags_file.pkl", debug=False, color=None, sim=False):
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
        :param bool debug: whether to print debug messages
        :param tuple(int) color: tuple of RGB values to define the color the robot should use for images outputs
        :param bool sim: are we controlling gazebo robots or simulating it with images

        """
        self._path = []
        self.robot_name = robot_name
        self.readable_robot_name = str(int(robot_name) + 1)
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
        self._current_task = None
        self.debug = debug
        self.color = color
        self.sim = sim
        self.joint_piston_controller = JointPistonNode()

        self._nav.setInitialPose(
            create_pose_stamped(
                nav=self._nav,
                x=self.current_pose.x * 1.,  # convert to a float
                y=self.current_pose.y * 1.,  # convert to a float
                z=self.current_pose.z * 1.,  # convert to a float
                roll=self.current_pose.roll * 1.,  # convert to a float
                pitch=self.current_pose.pitch * 1.,  # convert to a float
                yaw=90.,  # convert to a float, hardcode to face up
            )
        )

    def plan_path(self):
        """
        Plan full path from start to end using A* and avoiding obstacles and store in _path class variable

        :return: bool: Did we plan successfully
        """
        start_time = time.time()
        parent_dict = {}
        if not self.end_pose:
            print("No end pose for robot {} so a path could not be planned".format(self.readable_robot_name))
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
        cost_multiplier = 1  # Apply a cost modifier if we move onto a shelf node.  Used below.
        if theta_base == 0:  # this is right
            coordinates.append([(x_base + movement, y_base, theta_base), cost_multiplier])
        elif theta_base == 90:  # this is up
            coordinates.append([(x_base, y_base + movement, theta_base), cost_multiplier])
        elif theta_base == 180:  # this is left
            coordinates.append([(x_base - movement, y_base, theta_base), cost_multiplier])
        elif theta_base == 270:  # this is down (change to -90 in quaternion function)
            coordinates.append([(x_base, y_base - movement, theta_base), cost_multiplier])

        # correct turns in edge cases
        ccw_turn = theta_base - 90  # 90 degrees
        if ccw_turn < 0:
            ccw_turn = 270  # 270 degrees
        cw_turn = theta_base + 90  # 90 degrees
        if cw_turn > 270:  # 270 degrees
            cw_turn = 0

        # add possible turns
        cost_multiplier = 0  #  Don't apply additional cost of getting onto a shelf node if we're already there.  turning is ok.
        coordinates.append([(x_base, y_base, ccw_turn), cost_multiplier])
        coordinates.append([(x_base, y_base, cw_turn), cost_multiplier])

        for pose_tuple, cost_multiplier in coordinates:
            x, y, _ = pose_tuple
            # Ensure that the location has a valid tag and that the orchestrator has not locked the point
            if not self.orchestrator.is_pt_locked((x, y)) and (x, y, 0, 0, 0, 0) in self.tags.values():
                cost = motion_planner.cost(pose_tuple, self.end_pose.get_2d_pose())
                # If the location is where a shelf is, increase the cost significantly to try and avoid going through here unless absolutely necessary.
                if (x, y, 0, 0, 0, 0) in list(self.orchestrator.shelves.values()):
                    cost = cost + 5000*cost_multiplier
                all_possible_actions.append((pose_tuple, cost))
        
        return all_possible_actions

    def get_next_two_points(self):
        """
        Returns the next two path points
        :return: tuple(python_controllers.src.helpers.Pose | None): return next 2 points of the path if they exist.  Only x and y coordinates.
        """

        # If the robot is not at the end pose and we do not have a planned path, then plan one.
        if self.current_pose != self.end_pose and not self._path:
            self.plan_path()

        if len(self._path) > 1:
            first_point = Pose(x=self._path[0][0], y=self._path[0][1], yaw=0)
            second_point = Pose(x=self._path[1][0], y=self._path[1][1], yaw=0)
            return first_point, second_point
        elif len(self._path) == 1:
            first_point = Pose(x=self._path[0][0], y=self._path[0][1], yaw=0)
            return first_point, None
        else:
            return None, None

    def move_robot(self, sim):
        # if the robot is at a charge location task, then spend the cycle charging
        if self.current_pose == self.charge_locations[int(self.robot_name)] and not self._current_task:
            if self.debug:
                print("Robot {} is spending the cycle charging".format(self.readable_robot_name))
            self.battery_charge.charge_battery(BatteryCharge.CHARGE_PER_CYCLE)
        else:
            # Always drain the battery by this amount
            if self.battery_charge.drain_battery(BatteryCharge.DRAIN_PER_CYCLE):
                # print("Robot {} is out of charge".format(self.readable_robot_name))
                pass

            if not self._path:
                path_was_planned = False
                self.update_current_task()
                if self.end_pose and self.current_pose != self.end_pose:
                    path_was_planned = self.plan_path()  # returns True if path planned successfully
                if not path_was_planned:
                    # No path exists and we were not able to plan one, are already at the end pose, or don't have an end pose set
                    print("Robot {} did not move this cycle".format(self.readable_robot_name))
                    return
            # Since we're going to move, get the next waypoint and drain the battery for the upcoming move cycle
            # motion planner path uses tuple of floats (x, y, theta). So convert this back to a pose.
            next_path_pose = self._path.pop(0)
            next_path_pose = Pose(x=next_path_pose[0], y=next_path_pose[1], yaw=next_path_pose[2])
            if self.battery_charge.drain_battery(BatteryCharge.DRAIN_PER_MOVE):
                # print("Robot {} is out of charge".format(self.readable_robot_name))
                pass

            if not sim:
                # Set our demo's initial pose
                # http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html
                next_pose_stamped = create_pose_stamped(
                    nav=self._nav,
                    x=next_path_pose.x * 1.,  # convert to a float
                    y=next_path_pose.y * 1.,  # convert to a float
                    z=next_path_pose.z * 1.,  # convert to a float
                    roll=next_path_pose.roll * 1.,  # convert to a float
                    pitch=next_path_pose.pitch * 1.,  # convert to a float
                    yaw=next_path_pose.z * 1.,  # convert to a float
                )
                # http: // wiki.ros.org / tf2 / Tutorials / Quaternions
                # https: // answers.unity.com / questions / 147712 / what - is -affected - by - the - w - in -quaternionxyzw.html
                # https://www.programcreek.com/python/example/70252/geometry_msgs.msg.PoseStamped
                self._nav.goToPose(next_pose_stamped)
                while not self._nav.isTaskComplete():
                    feedback = self._nav.getFeedback()
                    print(feedback)
                    # if feedback.navigation_duration > 600:
                    #     self._nav.cancelTask()

                result = self._nav.getResult()
                print(result)
                print(next_pose_stamped)

            self.current_pose = next_path_pose

            for observer in self.observers:
                observer.__call__(self.current_pose)

        self.update_current_task()

    def update_current_task(self):
        # if a task exists, update its status if necessary
        if self._current_task:
            # mark that the robot has reached the shelf
            if self.current_pose == self._current_task.pick_up_location:
                if not self.sim:
                    self.joint_piston_controller.publish_piston_up()
                print("Robot {} has picked up the shelf".format(self.readable_robot_name))
                self.orchestrator.shelves[self._current_task.shelf_name] = None
                self.has_shelf = True
                self._current_task.has_shelf = True
                self.end_pose = self._current_task.drop_off_location  # Update end pose to drop off location
                self.orchestrator.reset_shelf_leg_locks()
            elif self.current_pose == self._current_task.drop_off_location:
                if not self.sim:
                    self.joint_piston_controller.publish_piston_down()
                print("Robot {} has completed it's task".format(self.readable_robot_name))
                self.orchestrator.shelves[self._current_task.shelf_name] = self.current_pose.get_6d_pose()
                self._current_task = None
                # Update end pose to charge station unless new tasking overwrites this
                self.end_pose = self.charge_locations[int(self.robot_name)]
                self.orchestrator.reset_shelf_leg_locks()

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
        The robot is considered "done" and available for tasking if it does not have a current task

        :return: bool is_done:
        """
        return not self._current_task


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

    def assign_new_task(self, task):
        """
        Get a new task, set the pick up point as the new end pose, plan the path

        :param RobotTask task: RobotTask instance
        """
        self._current_task = task
        self.end_pose = self._current_task.pick_up_location
        self.plan_path()

