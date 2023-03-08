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
    def initialize(self, x_i, x_g):
        q = queue.PriorityQueue()
        return super(AStarPlanner, self).initialize(q, x_i, initial_cost=self.cost(x_i[0], x_i[1], x_g[0], x_g[1]))
        # q.put([, x_i])
        # list_of_locations = {x_i: True}
        # return q, list_of_locations

    def cost(self, x_i, y_i, x_g, y_g, counter=0):
        return abs(x_i - x_g) + abs(x_i - x_g)
