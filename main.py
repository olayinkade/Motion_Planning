import time
import rrt
from domain_simulation import DomainSimulation


def main(argv=None):
    domain = DomainSimulation()
    domain.place_obstacles(5)
    domain.assign_initial_and_goal_positions()
    domain.print_map()
    print('> FINDING PATH USING RRT ALGORITHM...')
    print('RRT result:')
    start_time = time.time()
    rrt.rrt_explore(domain)
    end_time = time.time()
    domain.print_map()
    print('RRT took: ' + str(end_time - start_time) + 'seconds.')


if __name__ == '__main__':
    main()
