import queue
import random


class BaseMotionPlanner(object):
    def initialize(self, q, current_pose, initial_cost=0):
        q.put([initial_cost, current_pose])
        list_of_locations = {current_pose: True}
        return q, list_of_locations

    def append_action(self, q, possible_action, cost=0):
        q.put([cost, possible_action])
        return q

    def get_next(self, q):
        current_q = q.get_nowait()
        current_cost = current_q[0]
        x = current_q[1]
        return x, q, current_cost

    def cost(self, current_pose, goal_pose, counter=0):
        return 0


class AStarPlanner(BaseMotionPlanner):
    def initialize(self, current_pose, goal_pose):
        q = queue.PriorityQueue()
        return super(AStarPlanner, self).initialize(q, current_pose, initial_cost=self.cost(current_pose, goal_pose))

    def cost(self, current_pose, goal_pose, counter=0):
        x_current, y_current, theta_current = current_pose
        x_goal, y_goal, theta_goal = goal_pose
        return pow(abs(x_current - x_goal),2) + pow(abs(y_current - y_goal),2) + abs(theta_current - theta_goal)/90


class BreadthFirstSearchPlanner(BaseMotionPlanner):
    def initialize(self, current_pose, goal_pose):
        q = queue.Queue()
        return super(BreadthFirstSearchPlanner, self).initialize(q, current_pose)


class DepthFirstSearchPlanner(BaseMotionPlanner):
    def initialize(self, current_pose, goal_pose):
        q = queue.LifoQueue()
        return super(DepthFirstSearchPlanner, self).initialize(q, current_pose)


class RandomPlanner(BaseMotionPlanner):
    def initialize(self, current_pose, goal_pose):
        q = queue.PriorityQueue()
        return super(RandomPlanner, self).initialize(q, current_pose)

    def cost(self, current_pose, goal_pose, counter=0):
        return random.randint(0, 128*128)
