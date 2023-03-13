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
        self.request_queue = []
        # this set holds the ids of robots
        # waiting to plan a path. Deadlocked robots
        # also get placed here to wait for a new path
        self.waiting_robots = set()

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
        self.waiting_robots.add(robot_name)
        # self.robots[robot_name].plan_path()
        # first, second = self.robots[robot_name].get_next_two_points()
        # self.lock_cells(self.robots[robot_name], first, second)
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
        robots_to_move = [(r, self.robots[r]) for r in self.robots if not self.robots[r].is_done()]
        for id, robot in robots_to_move:
            # unlock reserved pts
            for pt in robot.locked_cells:
                if pt in self.locked:
                    self.locked.remove(pt)
            robot.locked_cells.clear()  

            # check if the robot is waiting
            # for a path. If so, try to plan
            # if we still can't find a path, skip this iteration
            if id in self.waiting_robots:
                success = robot.plan_path()
                if success:
                    self.waiting_robots.remove(id)
                else:
                    self.on_deadlock(robot.current_pose)
                    continue

            # identify the next two pts we need to
            # lock for this robot
            first, second = robot.get_next_two_points()
            # if either is already reserved for another robot
            # we need to replan the path
            if self.is_pt_locked(first) or self.is_pt_locked(second):                
                self.on_deadlock(robot.current_pose)
                self.waiting_robots.add(id)
                continue
            
            # once an available path has been found,
            # reserve the first and second pts for this
            # robot
            self.lock_cells(robot, first, second)
            robot.move_robot()
            # if we're at the shelf, turn around and go
            # back to charging/pickup station
            if robot.current_pose == robot.shelf_pose:
                robot.end_pose = robot.charging_station
                self.waiting_robots.add(id)
            # if we're back at the pickup station, and there's still
            # requests left, pop the next request
            elif robot.is_done() and len(self.request_queue) > 0:
                next = self.request_queue.pop(0)
                print(f'Requests left: {len(self.request_queue)}')
                robot.set_new_endpoint(next)
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


