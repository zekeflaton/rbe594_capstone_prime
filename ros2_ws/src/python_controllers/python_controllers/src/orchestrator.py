import os
from PIL import Image as im
import numpy as np
import random

import rclpy
from python_controllers.src.robot import Robot
from python_controllers.src.helpers import (
    BatteryCharge,
    Pose,
    RobotTask
)
from python_controllers.src.tag_locations import tags



class Orchestrator(object):
    def __init__(self,
                 shelves,
                 charge_locations,
                 size,
                 shelf_color,
                 charge_location_color,
                 motion_planner=None,
                 metrics_file_path=None,
                 debug=False,
                 orch_output_filepath=None,
                 sim=False,
                 extra_random_requests_to_make=0
                 ):
        """
        Initialize the orchestrator

        :param dict shelves: dictionary of {shelf name:shelf location}
        :param list(python_controllers.src.helpers.Pose) charge_locations: list of poses (x, y, theta) describing charge locations
        :param tuple(float) size: size of the map (x, y)
        :param Tuple(int) shelf_color: RGB tuple
        :param Tuple(int) charge_location_color: RGB tuple
        :param BaseMotionPlanner | None motion_planner: Optional override for a motion planner class
        :param str | None metrics_file_path: Optional file path to save metrics
        :param bool debug: whether to print debug messages
        :param str | None orch_output_filepath: filepath of where to save the images
        :param bool sim: are we controlling gazebo robots or simulating it with images
        :param int extra_random_requests_to_make: How many extra requests to autogenerate once current tasks are done
        """
        # Shelves are never removed but their pose values can change to None if they are being carried
        self.shelves = shelves
        self.tags = tags
        self.size = size
        self.robots: dict[str, Robot] = {}
        self.locked = set()
        self.deadlock_observers = []
        self.deadlock_count = 0
        self.charge_locations = charge_locations
        self.request_queue = []
        self.orch_output_filepath = orch_output_filepath
        self.extra_random_requests = extra_random_requests_to_make

        # this set holds the ids of robots
        # waiting to plan a path. Deadlocked robots
        # also get placed here to wait for a new path
        self.waiting_robots = set()
        self.motion_planner = motion_planner
        self.metrics_file_path = metrics_file_path
        self.battery_estimate_buffer = 0.2
        self.debug = debug
        self.sim = sim

        # Lock the 4 corners around the shelves to prevent the motion planners from using those points where the legs sit
        self.locked_shelves = set()  # Tuple(x, y) of all locked shelf locations
        self.reset_shelf_leg_locks()

        rclpy.init()

        if orch_output_filepath:
            # variables needed for numpy image building
            self.shelf_color = shelf_color
            self.charge_location_color = charge_location_color
            self.min_x = -4
            self.min_y = -4
            self.max_x = 5
            self.max_y = 3
            self.x_offset = -1 * self.min_x  # How far we need to adjust coordinate frames to get to 0 as the min
            self.y_offset = -1 * self.min_y  # How far we need to adjust coordinate frames to get to 0 as the min
            self.image_buffer = 10
            self.image_array = np.zeros(
                [
                    (self.x_offset + self.max_x) * self.image_buffer + 1,
                    (self.y_offset + self.max_y) * self.image_buffer + 1,
                    3,
                ],
                dtype='uint8',
            ) * 255  # Depth of 3: x, y, RGB
            self.reset_image_array()
            self.move_cycle = 0

            # Make the results directory if it does not exist
            if not os.path.exists(self.orch_output_filepath):
                # Create a new directory because it does not exist
                os.makedirs(self.orch_output_filepath)
                print("The new directory is created!")

    def convert_x_to_img_x(self, x):
        return int((x + self.x_offset) * self.image_buffer)

    def convert_y_to_img_y(self, y):
        return int((y + self.y_offset) * self.image_buffer)

    def reset_image_array(self):
        for x in range(self.image_array.shape[0]):
            for y in range(self.image_array.shape[1]):
                self.image_array[x, y] = np.array([255, 255, 255])
        self.paint_shelves_to_image_array()

    def paint_charge_locations_to_image_array(self):
        for charge_location_pose in self.charge_locations:
            # print([self.convert_x_to_img_x(charge_location_pose.x), self.convert_y_to_img_y(charge_location_pose.y)])
            self.image_array[self.convert_x_to_img_x(charge_location_pose.x), self.convert_y_to_img_y(charge_location_pose.y)] = np.array(
                self.charge_location_color)

    def paint_shelves_to_image_array(self):
        for shelf_name, shelf_pose in self.shelves.items():
            if shelf_pose:
                self.image_array[self.convert_x_to_img_x(shelf_pose[0]), self.convert_y_to_img_y(shelf_pose[1])] = np.array(self.shelf_color)

    def save_current_state_to_image(self, future_cycles_to_include=None):

        self.reset_image_array()
        self.paint_shelves_to_image_array()
        self.paint_charge_locations_to_image_array()
        for i, r in self.robots.items():
            previous_waypoint = list(r.current_pose.get_2d_pose())
            previous_waypoint[0] = (previous_waypoint[0] + self.x_offset) * self.image_buffer
            previous_waypoint[1] = (previous_waypoint[1] + self.y_offset) * self.image_buffer
            previous_waypoint = [int(point) for point in previous_waypoint]
            # print("waypoint", previous_waypoint)
            self.image_array[previous_waypoint[0], previous_waypoint[1], :] = r.color

            for j, waypoint in enumerate(r._path):
                if future_cycles_to_include and j >= future_cycles_to_include:
                    break
                waypoint = list(waypoint)
                waypoint[0] = (waypoint[0] + self.x_offset) * self.image_buffer
                waypoint[1] = (waypoint[1] + self.y_offset) * self.image_buffer
                waypoint = [int(point) for point in waypoint]
                # print("waypoint", waypoint)
                x, y, theta = waypoint
                self.image_array[int(x), int(y)] = r.color
                if previous_waypoint:
                    # previous_waypoint = [int(point) for point in previous_waypoint]
                    prev_x, prev_y, _ = previous_waypoint
                    # print("prev_x: ", prev_x, " -- x: ", x)
                    # print("prev_y: ", prev_y, " -- y: ", y)
                    if prev_x != x:
                        min_paint = min(prev_x, x)
                        max_paint = max(prev_x, x)
                        for dx in range(min_paint, max_paint):
                            # print(dx, y)
                            self.image_array[int(dx), int(y)] = r.color
                    elif prev_y != y:
                        min_paint = min(prev_y, y)
                        max_paint = max(prev_y, y)
                        for dy in range(min_paint, max_paint):
                            # print(x, dy)
                            self.image_array[int(x), int(dy), :] = r.color
                previous_waypoint = waypoint

        warehouse_map_png = im.fromarray(self.image_array, mode='RGB')
        warehouse_map_png_resized = warehouse_map_png.resize((600, 600), im.NEAREST)
        warehouse_map_png_resized_rotated = warehouse_map_png_resized.rotate(90)  # origin is bottom left for warehouse map and top left for images.

        warehouse_map_png_resized_rotated.save(os.path.join(self.orch_output_filepath, 'warehouse_map_{}.png'.format(str(self.move_cycle).zfill(5))))

    def reset_shelf_leg_locks(self):
        """
        Reset all shelf locks.  Can do this when we pick up a shelf or set one down.
        :param shelf_name:
        :return:
        """
        # TODO: Can make this more dynamic such that we simply remove shelf points as long as another shelf doesnt have the same leg spot.
        for x, y in self.locked_shelves.copy():
            robot_locked_point = False
            for id, robot in self.robots.items():
                if (x, y) in robot.locked_cells:
                    robot_locked_point = True
            if not robot_locked_point:
                self.locked.remove((x, y))
            self.locked_shelves.remove((x, y))
        if self.locked_shelves:
            print("ERROR: reset_shelf_leg_locks")
            raise
        for shelf_name, shelf_pose in self.shelves.items():
            # print("shelf_name: ", shelf_name, " -- shelf_pose: ", shelf_pose)
            if shelf_pose:
                for x_adjust in [-0.5, 0.5]:
                    for y_adjust in [-0.5, 0.5]:
                        x, y, z, roll, pitch, yaw = shelf_pose
                        point_surrounding_shelf_pose = Pose(x+x_adjust, y+y_adjust, 0, 0, 0, 0)
                        # print("point_surrounding_shelf_pose.get_6d_pose(): ", point_surrounding_shelf_pose.get_6d_pose())
                        # print("self.tags.values(): ", self.tags.values())
                        if point_surrounding_shelf_pose.get_6d_pose() in self.tags.values():
                            # print(point_surrounding_shelf_pose.get_xy())
                            self.locked_shelves.add(point_surrounding_shelf_pose.get_xy())
                            self.locked.add(point_surrounding_shelf_pose.get_xy())

    def add_robot(self, robot_name, initial_pose, end_pose=None, color=None):
        """

        :param int/str robot_name:
        :param initial_pose:
        :param end_pose:
        :param tuple(int) color: tuple of RGB values to define the color the robot should use for images outputs
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
            debug=self.debug,
            color=color
        )
        self.waiting_robots.add(robot_name)
        return self.robots[robot_name]

    def make_request(self, task_request):
        """
        Accept a task and add it to the queue to be assigned to a robot when it becomes available.

        :param python_controllers.src.hlpers.RobotTask task_request: task to append to the unassigned list
        :return:
        """
        self.request_queue.append(task_request)
        self.assign_tasks_for_robots()

    def on_deadlock(self, pose):
        for observer in self.deadlock_observers:
            observer.__call__(pose)

    def move_all(self):
        """
        Execute a move for all of the robots under the orchestrator
        control. If a collision is detected, replan the path.
        """
        self.move_cycle += 1

        # if there are requests in the queue, assign them out.
        self.assign_tasks_for_robots()

        # Move each robot while avoiding deadlocks using the strategy of locking the next 2 waypoints
        for id, robot in self.robots.items():

            # unlock reserved pts
            for pt in robot.locked_cells:
                if pt in self.locked:
                    self.locked.remove(pt)
            robot.locked_cells.clear()  

            # identify the next two pts we need to lock for this robot
            first, second = robot.get_next_two_points()
            # if either is already reserved for another robot we need to replan the path
            if (first and self.is_pt_locked(first.get_xy())) or (second and self.is_pt_locked(second.get_xy())):
                self.on_deadlock(robot.current_pose)
                print("robot {} deadlocked".format(robot.readable_robot_name))
                robot.plan_path()
                first, second = robot.get_next_two_points()

            # Once an available path has been found, reserve the first and second pts for this robot
            self.lock_cells(robot, first, second)
            robot.move_robot(self.sim)

            if robot.is_done() and (self.request_queue or self.extra_random_requests):

                # If no requests in queue but we want extra random requests, generate one
                if not self.request_queue and self.extra_random_requests > 0:
                    self.extra_random_requests -= 1
                    # Get list of all available drop off locations.  Start with full grid of options
                    # Then remove any grid locations that are a charge location in the for loops below
                    # Lastly remove any grid locations being used as an end point by current robot tasks several sections below ---->
                    available_drop_off_locations = []
                    for x in [float(j) / 10 for j in range(self.min_x*10, self.max_x*10+1, 5)]:
                        for y in [float(j) / 10 for j in range(self.min_y * 10, self.max_y * 10 + 1, 5)]:
                            p = Pose(x=x, y=y, yaw=0)
                            if p not in self.charge_locations and p.get_6d_pose() not in list(self.shelves.values()):
                                available_drop_off_locations.append(p)

                    # Get list of all available shelves.  Start with full shelf list
                    available_shelves = list(self.shelves.keys())

                    # Remove shelves and end poses currently being used by other tasks
                    # <----- Remove any available destination grid locations if they're currently being used
                    for r_name, r in self.robots.items():
                        if r._current_task:
                            available_shelves.remove(r._current_task.shelf_name)
                            if r._current_task.drop_off_location in available_drop_off_locations:
                                available_drop_off_locations.remove(r._current_task.drop_off_location)
                                self.remove_points_around_shelves(r._current_task.drop_off_location, available_drop_off_locations)

                    shelf_for_task = random.choice(available_shelves)
                    drop_off_location = random.choice(available_drop_off_locations)
                    task = RobotTask(
                        shelf_name=shelf_for_task,
                        drop_off_location=drop_off_location,
                        shelves=self.shelves
                    )
                    self.make_request(task)
                else:
                    next_request = self.request_queue.pop(0)
                    print(f'Requests left: {len(self.request_queue)}')
                    robot.assign_new_task(next_request)

        if self.orch_output_filepath:
            self.save_current_state_to_image(1)

    def remove_points_around_shelves(self, shelf_pose, points_list):
        """

        :param Pose shelf_pose: pose of the shelf to clear surrounding points
        :param list(Pose) points_list:
        :return: list(Pose)
        """
        for x_adjust in [-0.5, 0.5]:
            for y_adjust in [-0.5, 0.5]:
                x, y, z, roll, pitch, yaw = shelf_pose.get_6d_pose()
                point_surrounding_shelf_pose = Pose(x + x_adjust, y + y_adjust, 0, 0, 0, 0)
                # print("point_surrounding_shelf_pose.get_6d_pose(): ", point_surrounding_shelf_pose.get_6d_pose())
                # print("self.tags.values(): ", self.tags.values())
                if point_surrounding_shelf_pose.get_6d_pose() in self.tags.values():
                    if point_surrounding_shelf_pose in points_list:
                        points_list.remove(point_surrounding_shelf_pose)
        return shelf_pose

    def lock_cells(self, robot, first, second=None):
        """
        This method should only be called from within the orchestrator as a convenience
         for locking pts.  Locks the x and y coordinates for path planning purposes
        :param python_controllers.src.robot.Robot robot: robot that's using the poses
        :param python_controllers.src.helpers.Pose first: first pose
        :param python_controllers.src.helpers.Pose | None second: second pose if available
        """
        self.locked.add(robot.current_pose.get_xy())
        robot.locked_cells.append(robot.current_pose.get_xy())
        if first is not None:
            self.locked.add(first.get_xy())
            robot.locked_cells.append(first.get_xy())
        if second is not None:
            self.locked.add(second.get_xy())
            robot.locked_cells.append(second.get_xy())

    def is_done(self):
        '''Test if the current pose matches the goal pose'''
        alldone = [self.robots[r].is_done() for r in self.robots]
        return all(alldone)

    def are_all_robots_at_home_without_task(self):
        all_robots_at_home_without_task = True
        for r_name, r in self.robots.items():
            all_robots_at_home_without_task = (r.current_pose == r.charge_locations[int(r_name)]) and not r._current_task
            # If any single robot is not home, return false
            if not all_robots_at_home_without_task:
                return all_robots_at_home_without_task
        return all_robots_at_home_without_task

    def is_pt_locked(self, pt):
        """
        Test if a pt is either a shelf or reserved for a robot
        :param tuple(int) pt: (x, y)
        :return: bool
        """
        # print(pt)
        # print(self.locked)
        if pt is None:
            return False
        else:
            return pt in self.locked

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
        # if self.debug: print("start find_robot_for_task")
        robot_to_use = None
        best_robot_distance_to_cover = None
        for robot in self.robots.values():
            if robot._current_task:
                continue
            # Get the manhattan distance of the robots currnet pose to the first task plus the tasks pick up point plus
            # the distance of the tasks pick up point to the drop off point.
            distance_to_cover = Pose.manhattan_distance(robot.current_pose, task.pick_up_location) + Pose.manhattan_distance(task.pick_up_location, task.drop_off_location)

            # Estimate battery usage with a buffer for unexpected obstacles or deadlocks
            battery_usage_estimate = (BatteryCharge.DRAIN_PER_CYCLE + BatteryCharge.DRAIN_PER_MOVE) * distance_to_cover * (1 + self.battery_estimate_buffer)
            # if self.debug: print("battery_usage_estimate", battery_usage_estimate)
            # if self.debug: print("robot.battery_charge.battery_charge", robot.battery_charge.battery_charge)
            # Only consider the robot if it has enough battery to get to its destination
            if robot.battery_charge.battery_charge >= battery_usage_estimate:
                # If we do not have a robot set or this robot is closer to the pick up point, then select this as the best option
                if not robot_to_use or distance_to_cover < best_robot_distance_to_cover:
                    robot_to_use = robot
                    best_robot_distance_to_cover = distance_to_cover
        # if self.debug: print("robot_to_use", robot_to_use)
        # if self.debug: print("end find_robot_for_task")
        return robot_to_use

    def assign_tasks_for_robots(self):
        # if self.debug: print("start assign_tasks_for_robots")
        # if self.debug: print("orchestrator.request_queue", self.request_queue)
        if self.request_queue:
            # Iterate through number of robots are requests, whichever is less
            for i in range(min(len(self.robots), len(self.request_queue))):
                # Check to see if any robots are available for tasking and assign tasks off the request queue
                robot_for_tasking = self.find_robot_for_task(self.request_queue[0])
                if robot_for_tasking:
                    robot_for_tasking.assign_new_task(self.request_queue.pop(0))
                else:
                    # If no robots are available for tasking, break out of this loop
                    # if self.debug: print("end assign_tasks_for_robots")
                    break
        # if self.debug: print("end assign_tasks_for_robots")

