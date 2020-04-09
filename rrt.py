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

    def set_parent(self, new_parent):
        self.parent = new_parent
        if self.parent is None:
            self.distance = float('inf')
        else:
            self.distance = new_parent.distance + new_parent.calculate_distance_to(self.location)

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
                change = i * curr_step_length / (curr_step_length + 1)
                r = self.location[0] + int(change * row_move)
                c = self.location[1] + int(change * col_move)
                # print("row move " + str(row_move) + ' - ' + str(col_move))
                # print('** ' + str(change) + ' - ' + str(r) + ' - ' + str(c))
                if domain_simulation.map[r][c] == 'o':
                    return new_tree_node
                else:
                    new_tree_node = RRTNode(None, (r, c))
            return new_tree_node


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


def rrt_explore(domain_simulation: DomainSimulation):
    rrt_tree = RRTTree(domain_simulation.initial_pos, domain_simulation.goal_pos)
    rrt_tree.initial.distance = 0
    path_found = is_path_found(domain_simulation, rrt_tree)
    while not path_found:
        # print('HERE')
        random_point = generate_random_point(domain_simulation)
        shortest_distance, nearest_node = float('inf'), None
        for curr_node in rrt_tree.nodes:
            distance = curr_node.calculate_distance_to(random_point.location)
            if distance < shortest_distance:
                shortest_distance = distance
                nearest_node = curr_node
        new_added_node = nearest_node.check_obstacle_in_between(domain_simulation, random_point.location)
        if new_added_node is not None:
            new_added_node.set_parent(nearest_node)
            rrt_tree.nodes.append(new_added_node)

        path_found = is_path_found(domain_simulation, rrt_tree)

    path = rrt_tree.get_path_backward()

    point = path.pop()
    for i in range(1, len(path) + 1):
        curr = path.pop()
        show_path(domain_simulation, point.location, curr.location)
        point = curr
    domain_simulation.print_map()


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
        return path_found


def generate_random_point(domain_simulation: DomainSimulation) -> RRTNode:
    stop = False
    random_location = [0, 0]
    # random_r, random_c = 0, 0
    while not stop:
        random_location[0] = int(random.uniform(0, SIZE - 1))
        random_location[1] = int(random.uniform(0, SIZE - 1))
        if domain_simulation.map[random_location[0]][random_location[1]] == ' ':
            stop = True
    return RRTNode(None, tuple(random_location))


def show_path(domain_simulation: DomainSimulation, point_1: tuple, point_2: tuple):
    if point_1[0] != point_2[0]:
        slope = calculate_slope(point_1, point_2, True)
        if point_1[0] >= point_2[0]:
            for i in range(0, point_1[0] - point_2[0]):
                if domain_simulation.map[i+point_2[0]][int(point_1[1] + slope * ((i+point_2[0]) - point_1[0]))] == ' ':
                    domain_simulation.map[i+point_2[0]][int(point_1[1] + slope * ((i+point_2[0]) - point_1[0]))] = '<'
        else:
            for i in range(0, point_2[0] - point_1[0]):
                if domain_simulation.map[i+point_1[0]][int(point_2[1] + slope * ((i+point_1[0]) - point_2[0]))] == ' ':
                    domain_simulation.map[i+point_1[0]][int(point_2[1] + slope * ((i+point_1[0]) - point_2[0]))] = '>'
    if point_1[1] != point_2[1]:
        slope = calculate_slope(point_1, point_2, False)
        if point_1[1] >= point_2[1]:
            for i in range(0, point_1[1] - point_2[1]):
                if domain_simulation.map[int(point_1[0] + slope * ((i+point_2[1]) - point_1[1]))][i+point_2[1]] == ' ':
                    domain_simulation.map[int(point_1[0] + slope * ((i+point_2[1]) - point_1[1]))][i+point_2[1]] = '^'
        else:
            for i in range(0, point_2[1] - point_1[1]):
                if domain_simulation.map[int(point_2[0] + slope * ((i+point_1[1]) - point_2[1]))][i+point_1[1]] == ' ':
                    domain_simulation.map[int(point_2[0] + slope * ((i+point_1[1]) - point_2[1]))][i+point_1[1]] = 'v'


# calculate the slope between two points
def calculate_slope(point_1: tuple, point_2: tuple, x_diff: bool) -> float:
    if x_diff:
        return (point_2[1] - point_1[1]) / (point_2[0] - point_1[0])
    else:
        return (point_2[0] - point_1[0]) / (point_2[1] - point_1[1])
