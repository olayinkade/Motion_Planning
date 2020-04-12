import random
import numpy as np


SIZE = 100


class DomainSimulation:

    # initialize the domain map, assume the size is 100
    def __init__(self):
        self.initial_pos = (0, 0)
        self.goal_pos = (0, 0)
        self.map = [[' ' for i in range(SIZE)] for j in range(SIZE)]

    # randomly generate obstacles for the map, given the number of required obstacles.
    def place_obstacles(self, num_obstacles: int):
        obstacles = []
        for _ in range(num_obstacles):
            obstacle_centre = (random.randint(0, SIZE), random.randint(0, SIZE))
            obstacle_size = (random.randint(10, 15), random.randint(10, 15))
            half_width, half_height = int(obstacle_size[0]/2), int(obstacle_size[1]/2)
            # the limit/borders of the obstacle in the form of (top, bottom, left, right)
            obstacle_borders = (max(0, obstacle_centre[0] - half_width), min(SIZE, obstacle_centre[0] + half_width),
                                max(0, obstacle_centre[1] - half_height), min(SIZE, obstacle_centre[1] + half_height))
            obstacles.append((obstacle_centre, half_width, half_height))
            for r in range(obstacle_borders[0], obstacle_borders[1]):
                for c in range(obstacle_borders[2], obstacle_borders[3]):
                    self.map[r][c] = 'o'
        return obstacles

    # assign two available cells to be initial point and goal in a uniform distribution fashion
    def assign_initial_and_goal_positions(self):
        # I for the initial point, G for the goal
        self.initial_pos = self.assign_new_available_point()
        self.map[self.initial_pos[0]][self.initial_pos[1]] = 'I'

        self.goal_pos = self.assign_new_available_point()
        self.map[self.goal_pos[0]][self.goal_pos[1]] = 'G'

    # try to find an empty cell, meaning a cell that is not the initial point, not the goal and not inside any obstacles
    def assign_new_available_point(self) -> tuple:
        row = int(random.uniform(0, SIZE - 1))
        col = int(random.uniform(0, SIZE - 1))
        while self.map[row][col] != ' ':
            row = int(random.uniform(0, SIZE - 1))
            col = int(random.uniform(0, SIZE - 1))
        return row, col

    def print_map(self):
        print()
        for j in range(-1, SIZE + 1):
            for i in range(-1, SIZE + 1):
                if (i == -1 or i == SIZE) and (j == -1 or j == SIZE):
                    print('+', end='')
                elif i == -1 or i == SIZE:
                    print('|', end='')
                elif j == -1 or j == SIZE:
                    print('-', end='')
                else:
                    print(self.map[i][j], end='')
            print()
