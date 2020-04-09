import math
import random
from typing import List

from domain_simulation import DomainSimulation, SIZE


class RRTNode:
    # location is a tuple in the form of: (row number, column number)
    def __init__(self, parent, location: tuple):
        self.parent = parent
        self.location = location
        self.distance = float('inf')  # distance from the initial point

    # get distance from the current point to the new point
    # new point is a tuple in the form of (row, col)
    def calculate_distance_to(self, new_point: tuple):
        return math.sqrt(math.pow(self.location[0] - new_point[0], 2) + math.pow(self.location[1] - new_point[1], 2))

    # check to see if there is any obstacle between current point and a new point
    # return None if it's next to an obstacle, return the furthest possible point on path otherwise (max step is 5)
    def check_obstacle_in_between(self, domain_simulation: DomainSimulation, end_point: tuple):
        new_tree_node = None
        distance = self.calculate_distance_to(end_point)
        curr_step_length = int(min(distance, 5))
        if curr_step_length == 0:
            return new_tree_node
        else:
            row_move = (end_point[0] - self.location[0]) / distance
            col_move = (end_point[1] - self.location[1]) / distance
            for i in range(curr_step_length + 1):
                r, c = self.calculate_new_coordinate(row_move, col_move, curr_step_length, i)
                if domain_simulation.map[r][c] == 'o':
                    return new_tree_node
                else:
                    new_tree_node = RRTNode(None, (r, c))
            return new_tree_node

    def calculate_new_coordinate(self, row_move: float, col_move: float, curr_step_length: int, index: int) -> tuple:
        change = index * curr_step_length / (curr_step_length + 1)
        return self.location[0] + int(change * row_move), self.location[1] + int(change * col_move)

    def set_parent(self, new_parent):
        self.parent = new_parent

    def set_distance(self):
        if self.parent is None:
            self.distance = float('inf')
        else:
            self.distance = self.parent.distance + self.parent.calculate_distance_to(self.location)


class RRTTree:
    def __init__(self, initial: tuple, goal: tuple):
        self.initial = RRTNode(None, initial)
        self.goal = RRTNode(None, goal)
        self.nodes = [self.initial]

    def get_path_backward(self) -> List:
        path = []
        curr_node = self.goal
        while curr_node is not None:
            path.append(curr_node)
            curr_node = curr_node.parent
        return path

    def add_node(self, new_node):
        self.nodes.append(new_node)


# RRT algorithm, given the domain simulation with initial point, goal and obstacles.
def rrt_explore(domain_simulation: DomainSimulation):
    rrt_tree = RRTTree(domain_simulation.initial_pos, domain_simulation.goal_pos)
    rrt_tree.initial.distance = 0
    path_found = is_path_found(domain_simulation, rrt_tree)

    while not path_found:
        random_point = generate_random_point(domain_simulation)
        shortest_distance, nearest_node = float('inf'), None

        for curr_node in rrt_tree.nodes:
            distance = curr_node.calculate_distance_to(random_point.location)
            if distance < shortest_distance:
                shortest_distance = distance
                nearest_node = curr_node
        new_added_node = nearest_node.check_obstacle_in_between(domain_simulation, random_point.location)

        if new_added_node is not None:
            set_new_tree_node(nearest_node, new_added_node, rrt_tree)

        path_found = is_path_found(domain_simulation, rrt_tree)

    connect_all_points(domain_simulation, rrt_tree)


def is_path_found(domain_simulation: DomainSimulation, rrt_tree: RRTTree) -> bool:
    last_node = rrt_tree.nodes[-1]
    distance = last_node.calculate_distance_to(rrt_tree.goal.location)
    curr_step_max = 5
    if distance > curr_step_max:
        return False
    else:
        path_found = True
        curr_step_max = int(distance)
        for i in range(curr_step_max):
            move_r = last_node.location[0] + int(i / distance)
            move_c = last_node.location[1] + int(i / distance)
            if domain_simulation.map[move_r][move_c] == 'o':
                path_found = False
                break
        if path_found:
            rrt_tree.goal.set_parent(last_node)
            rrt_tree.goal.set_distance()
        return path_found


# generate a new random point on the map to be considered for the tree
def generate_random_point(domain_simulation: DomainSimulation) -> RRTNode:
    stop = False
    random_location = [0, 0]
    while not stop:
        random_location[0] = int(random.uniform(0, SIZE - 1))
        random_location[1] = int(random.uniform(0, SIZE - 1))
        if domain_simulation.map[random_location[0]][random_location[1]] == ' ':
            stop = True
    return RRTNode(None, tuple(random_location))


# draw out the path with appropriate arrow signs that represent the direction to take from initial point to goal
def show_path(domain_simulation: DomainSimulation, point_1: tuple, point_2: tuple):
    if point_1[0] != point_2[0]:
        direction = calculate_slope(point_1, point_2, True)
        if point_1[0] >= point_2[0]:
            for i in range(0, point_1[0] - point_2[0]):
                if domain_simulation.map[i+point_2[0]][int(point_1[1] + direction * ((i+point_2[0]) - point_1[0]))] == ' ':
                    domain_simulation.map[i+point_2[0]][int(point_1[1] + direction * ((i+point_2[0]) - point_1[0]))] = '<'
        else:
            for i in range(0, point_2[0] - point_1[0]):
                if domain_simulation.map[i+point_1[0]][int(point_2[1] + direction * ((i+point_1[0]) - point_2[0]))] == ' ':
                    domain_simulation.map[i+point_1[0]][int(point_2[1] + direction * ((i+point_1[0]) - point_2[0]))] = '>'
    if point_1[1] != point_2[1]:
        direction = calculate_slope(point_1, point_2, False)
        if point_1[1] >= point_2[1]:
            for i in range(0, point_1[1] - point_2[1]):
                if domain_simulation.map[int(point_1[0] + direction * ((i+point_2[1]) - point_1[1]))][i+point_2[1]] == ' ':
                    domain_simulation.map[int(point_1[0] + direction * ((i+point_2[1]) - point_1[1]))][i+point_2[1]] = '^'
        else:
            for i in range(0, point_2[1] - point_1[1]):
                if domain_simulation.map[int(point_2[0] + direction * ((i+point_1[1]) - point_2[1]))][i+point_1[1]] == ' ':
                    domain_simulation.map[int(point_2[0] + direction * ((i+point_1[1]) - point_2[1]))][i+point_1[1]] = 'v'


# calculate the slope between two points
def calculate_slope(point_1: tuple, point_2: tuple, x_diff: bool) -> float:
    if x_diff:
        return (point_2[1] - point_1[1]) / (point_2[0] - point_1[0])
    else:
        return (point_2[0] - point_1[0]) / (point_2[1] - point_1[1])


def set_new_tree_node(nearest_node: RRTNode, new_added_node: RRTNode, rrt_tree: RRTTree):
    new_added_node.set_parent(nearest_node)
    new_added_node.set_distance()
    rrt_tree.add_node(new_added_node)


# connect all the points in on found path between initial point and goal
def connect_all_points(domain_simulation: DomainSimulation, rrt_tree: RRTTree):
    path = rrt_tree.get_path_backward()
    last_connected_point = path.pop()
    for i in range(1, len(path) + 1):
        curr = path.pop()
        show_path(domain_simulation, last_connected_point.location, curr.location)
        last_connected_point = curr
