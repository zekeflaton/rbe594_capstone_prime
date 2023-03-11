import queue
import random
import math


class BaseMotionPlanner(object):
    def initialize(self, q, x_i, initial_cost=0):
        q.put([initial_cost, x_i])
        list_of_locations = {x_i: True}
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
        return pow(abs(x_current - x_goal),2) + pow(abs(y_current - y_goal),2) + abs(theta_current - theta_goal)/90
