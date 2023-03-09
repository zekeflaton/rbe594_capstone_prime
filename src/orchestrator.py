from robot import RobotPathPlanner


class Orchestrator(object):
    def __init__(self, shelves, size):
        self.shelves = shelves
        self.size = size
        self.robots: dict[str, RobotPathPlanner] = {}
        self.locked = set()
        self.deadloc_observers = []
        self.deadlock_count = 0

    def add_robot(self, robot_name, initial_pose, end_pose):
        """

        :param int/str robot_name:
        :param initial_pose:
        :param end_pose:
        :return:
        """
        self.robots[robot_name] = RobotPathPlanner(self.shelves, self, self.size[0], self.size[1], initial_pose,
                                                   end_pose)
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
                self.deadlock_count += 1
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


