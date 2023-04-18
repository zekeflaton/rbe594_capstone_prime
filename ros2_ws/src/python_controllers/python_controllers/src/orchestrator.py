import rclpy
from python_controllers.src.robot import Robot
from python_controllers.src.helpers import (
    get_point_from_pose,
    BatteryCharge,
    Pose
)
from python_controllers.src.tag_locations import tags



class Orchestrator(object):
    def __init__(self, shelves, charge_locations, size, motion_planner=None, metrics_file_path=None):
        """
        Initialize the orchestrator

        :param dict shelves: dictionary of {shelf name:shelf location}
        :param list(python_controllers.src.helpers.Pose) charge_locations: list of poses (x, y, theta) describing charge locations
        :param tuple(float) size: size of the map (x, y)
        :param BaseMotionPlanner | None motion_planner: Optional override for a motion planner class
        :param str | None metrics_file_path: Optional file path to save metrics
        """
        self.shelves = shelves
        self.tags = tags
        self.size = size
        self.robots: dict[str, Robot] = {}
        self.locked = set()
        self.deadlock_observers = []
        self.deadlock_count = 0
        self.charge_locations = charge_locations
        self.request_queue = []
        # this set holds the ids of robots
        # waiting to plan a path. Deadlocked robots
        # also get placed here to wait for a new path
        self.waiting_robots = set()
        self.motion_planner = motion_planner
        self.metrics_file_path = metrics_file_path
        self.battery_estimate_buffer = 0.2

        rclpy.init()


    def add_robot(self, robot_name, initial_pose, end_pose=None):
        """

        :param int/str robot_name:
        :param initial_pose:
        :param end_pose:
        :return:
        """
        self.robots[robot_name] = Robot(
            robot_name=robot_name,
            charge_locations=self.charge_locations,
            orchestrator=self,
            max_x=self.size[0],
            max_y=self.size[1],
            initial_pose=initial_pose,
            end_pose=end_pose,
            motion_planner=self.motion_planner,
            metrics_file_path=self.metrics_file_path,
        )
        self.waiting_robots.add(robot_name)
        return self.robots[robot_name]

    def make_request(self, end_pose):
        self.request_queue.append(end_pose)

    def on_deadlock(self, pose):        
        for observer in self.deadlock_observers:
            observer.__call__(pose)

    def move_all(self):
        """
        Execute a move for all of the robots under the orchestrator
        control. If a collision is detected, replan the path.
        """
        for i in range(len(self.robots)):
            # Check to see if any robots are available for tasking and assign tasks off the request queue
            robot_for_tasking = self.find_robot_for_task(self.request_queue[0])
            if robot_for_tasking:
                robot_for_tasking.assign_new_task(self.request_queue.pop(0))
            else:
                # If no robots are available for tasking, break out of this loop
                break

        # Move each robot while avoiding deadlocks using the strategy of locking the next 2 waypoints
        for id, robot in self.robots:

            # unlock reserved pts
            for pt in robot.locked_cells:
                if pt in self.locked:
                    self.locked.remove(pt)
            robot.locked_cells.clear()  

            # identify the next two pts we need to lock for this robot
            first, second = robot.get_next_two_points()
            # if either is already reserved for another robot we need to replan the path
            if self.is_pt_locked(first) or self.is_pt_locked(second):                
                self.on_deadlock(robot.current_pose)
                robot.plan_path()
                first, second = robot.get_next_two_points()

            # Once an available path has been found, reserve the first and second pts for this robot
            self.lock_cells(robot, first, second)
            robot.move_robot()

            if robot.is_done() and self.request_queue:
                next_request = self.request_queue.pop(0)
                print(f'Requests left: {len(self.request_queue)}')
                robot.assign_new_task(next_request)
                self.waiting_robots.add(id)

    def lock_cells(self, robot, first, second=None):
        """

        This method should only be called from within the orchestrator
        as a conveinience for locking pts
        """
        if first is not None:
            self.locked.add(get_point_from_pose(first))
            robot.locked_cells.append(get_point_from_pose(first))
        if second is not None:
            self.locked.add(get_point_from_pose(second))
            robot.locked_cells.append(get_point_from_pose(second))

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
        self.deadlock_observers.append(func)

    def unsubscribe_to_deadlock(self, func):
        self.deadlock_observers.remove(func)

    def find_robot_for_task(self, task):
        """

        :param RobotTask task: RobotTask instance
        :return: python_controllers.src.robot.Robot robot: Instance of the robot that's available for
                        tasking. If o robots have sufficient charge for the task, return None
        """
        robot_to_use = None
        best_robot_distance_to_cover = None
        for robot in self.robots:
            # Get the manhattan distance of the robots currnet pose to the first task plus the tasks pick up point plus
            # the distance of the tasks pick up point to the drop off point.
            distance_to_cover = Pose.manhattan_distance(robot.current_pose, task.pick_up_location) + Pose.manhattan_distance(task.pick_up_location, task.drop_off_location)

            # Estimate battery usage with a buffer for unexpected obstacles or deadlocks
            battery_usage_estimate = BatteryCharge.DRAIN_PER_CYCLE * distance_to_cover * (1 + self.battery_estimate_buffer)

            # Only consider the robot if it has enough battery to get to its destination
            if robot.battery_charge.battery_charge >= battery_usage_estimate:
                # If we do not have a robot set or this robot is closer to the pick up point, then select this as the best option
                if not robot_to_use or distance_to_cover < best_robot_distance_to_cover:
                    robot_to_use = robot
                    best_robot_distance_to_cover = distance_to_cover
        return robot_to_use


