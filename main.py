import time
import rrt
from domain_simulation import DomainSimulation
from fbsp_path_planning import *


def main(argv=None):
    num_obstacles = input('> Enter number of obstacles to be placed: ')
    while not num_obstacles.isdigit():
        print('ERROR!!! Number of obstacles needs to be an int. Please re-enter!')
        num_obstacles = input('> Enter number of obstacles to be placed: ')
    domain = DomainSimulation()
    obs_list = domain.place_obstacles(int(num_obstacles))
    domain.assign_initial_and_goal_positions()
    domain.print_map()

    # TODO: ***** FOR YINKA!!!! **** USE THE SAME domain TO TEST QUADTREE HERE
    # TODO: work with quadtree_domain instead of original one (domain)
    quadtree_domain = copy.deepcopy(domain)
    quad_tree_and_fbsp(obs_list,quadtree_domain)

    rrt_domain = copy.deepcopy(domain)
    print('> FINDING PATH USING RRT ALGORITHM...')
    print('RRT result:')
    start_time = time.time()
    rrt.rrt_explore(rrt_domain)
    end_time = time.time()
    rrt_domain.print_map()
    print('RRT took: ' + str(end_time - start_time) + 'seconds.')


def quad_tree_and_fbsp(obs_list, domain):
    width = 10.0
    height = 10.0

    pp = PathPlanningProblem(width, height, 20, 3.0, 1, obs_list)
    initial = (domain.initial_pos[0]/10.0, (10.0 - (domain.initial_pos[1]/10.0)))
    goals = (domain.goal_pos[0]/10.0, (10.0 - (domain.goal_pos[1]/10.0)))

    fig = plt.figure()
    ax = fig.add_subplot(1, 2, 1, aspect='equal')
    ax.set_xlim(0.0, width)
    ax.set_ylim(0.0, height)

    qtd = QuadTreeDecomposition(pp, 0.2)
    qtd.find_free_cell()
    initial_state = qtd.find_initial_state(initial, "free initial")

    goal_state = qtd.find_initial_state(goals, "free goal")
    print('> FINDING PATH USING QUADTREE ALGORITHM...')
    print('QUADTREE result:')
    start_time = time.time()
    astar(initial_state, goal_state, qtd.free_cell)
    end_time = time.time()

    print('QUADTREE took: ' + str(end_time - start_time) + 'seconds.')

    qtd.Draw(ax)
    n = qtd.CountCells()
    ax.set_title('Quadtree Decomposition\n{0} cells'.format(n))

    ax = fig.add_subplot(1, 2, 2, aspect='equal')
    ax.set_xlim(0.0, width)
    ax.set_ylim(0.0, height)

    for o in pp.obstacles:
        ax.add_patch(copy.copy(o.patch))

    bsp = BinarySpacePartitioning(pp, 0.2)
    bsp.find_free_cell()
    initial_state = bsp.find_initial_state(initial, "free initial")
    goal_state = bsp.find_initial_state(goals, "free goal")
    print('> FINDING PATH USING FBSP TREE ALGORITHM...')
    print('FBSP TREE result:')
    start_time = time.time()
    astar(initial_state, goal_state, bsp.free_cell)
    end_time = time.time()
    print('FBSP TREE took: ' + str(end_time - start_time) + 'seconds.')
    bsp.Draw(ax)
    n = bsp.CountCells()
    ax.set_title('FBSP Decomposition\n{0} cells'.format(n))
    plt.show()

if __name__ == '__main__':
    main()
