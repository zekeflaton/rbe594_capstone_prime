from .robot import Robot, get_point_from_pose


class Orchestrator(object):
    def __init__(self, shelves, size):
        self.shelves = shelves
        self.size = size
        self.robots: dict[str, Robot] = {}
        self.locked = set()
        self.deadlock_observers = []
        self.deadlock_count = 0
        self.charge_locations = []

    def add_robot(self, robot_name, initial_pose, end_pose):
        """

        :param int/str robot_name:
        :param initial_pose:
        :param end_pose:
        :return:
        """
        self.robots[robot_name] = Robot(
            robot_name=robot_name,
            obstacles=self.shelves,
            charge_locations=self.charge_locations,
            orchestrator=self,
            max_x=self.size[0],
            max_y=self.size[1],
            initial_pose=initial_pose,
            end_pose=end_pose
)
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
                if pt in self.locked:
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
                self.deadlock_count += 1
                for observer in self.deadlock_observers:
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


