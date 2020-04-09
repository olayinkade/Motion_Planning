import copy
import time
import rrt
from domain_simulation import DomainSimulation


def main(argv=None):
    num_obstacles = input('> Enter number of obstacles to be placed: ')
    while not num_obstacles.isdigit():
        print('ERROR!!! Number of obstacles needs to be an int. Please re-enter!')
        num_obstacles = input('> Enter number of obstacles to be placed: ')
    domain = DomainSimulation()
    domain.place_obstacles(int(num_obstacles))
    domain.assign_initial_and_goal_positions()
    domain.print_map()

    # TODO: ***** FOR YINKA!!!! **** USE THE SAME domain TO TEST QUADTREE HERE
    # TODO: work with quadtree_domain instead of original one (domain)
    quadtree_domain = copy.deepcopy(domain)

    rrt_domain = copy.deepcopy(domain)
    print('> FINDING PATH USING RRT ALGORITHM...')
    print('RRT result:')
    start_time = time.time()
    rrt.rrt_explore(rrt_domain)
    end_time = time.time()
    rrt_domain.print_map()
    print('RRT took: ' + str(end_time - start_time) + 'seconds.')


if __name__ == '__main__':
    main()
