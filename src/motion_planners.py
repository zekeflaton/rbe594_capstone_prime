import queue
import random
import math


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

    def cost(self, x, y, x_g, y_g, counter):
        return 0


class AStarPlanner(BaseMotionPlanner):
    def initialize(self, current_pose, goal_pose):
        q = queue.PriorityQueue()
        return super(AStarPlanner, self).initialize(q, current_pose, initial_cost=self.cost(current_pose, goal_pose))

    def cost(self, current_pose, goal_pose, counter=0):
        x_current, y_current, theta_current = current_pose
        x_goal, y_goal, theta_goal = goal_pose
        return abs(x_current - x_goal) + abs(y_current - y_goal) + abs(theta_current - theta_goal)/90


class BreadthFirstSearchPlanner(BaseMotionPlanner):
    def initialize(self, current_pose, goal_pose):
        q = queue.Queue()
        return super(BreadthFirstSearchPlanner, self).initialize(q, current_pose)


class DepthFirstSearchPlanner(BaseMotionPlanner):
    def initialize(self, current_pose, goal_pose):
        q = queue.LifoQueue()
        return super(DepthFirstSearchPlanner, self).initialize(q, current_pose)


class DijkstrasPlanner(BaseMotionPlanner):
    def initialize(self, current_pose, goal_pose):
        q = queue.PriorityQueue()
        return super(DijkstrasPlanner, self).initialize(q, current_pose)

    def cost(self, x, y, x_g, y_g, counter):
        # Moving right has cost of 1
        # Moving left has cost of 2
        # Moving up has cost of 3
        # Moving down has cost of 4
        return counter


class RandomPlanner(BaseMotionPlanner):
    def initialize(self, current_pose, goal_pose):
        q = queue.PriorityQueue()
        return super(RandomPlanner, self).initialize(q, current_pose)

    def cost(self, x, y, x_g, y_g, counter):
        return random.randint(0, 128*128)
    