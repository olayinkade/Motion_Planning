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

    quadtree_domain = copy.deepcopy(domain)
    quad_tree_and_fbsp(obs_list,quadtree_domain, 'QUADTREE')
    quad_tree_and_fbsp(obs_list, quadtree_domain, 'FBSP TREE')
    
    rrt_domain = copy.deepcopy(domain)
    print('> FINDING PATH USING RRT ALGORITHM...')
    print('RRT result:')
    start_time = time.time()
    rrt_domain = rrt.rrt_explore(rrt_domain)
    end_time = time.time()
    rrt_domain.print_map()
    print('RRT took: ' + str(end_time - start_time) + 'seconds.')

    draw_map(quadtree_domain, obs_list, rrt_domain.map)


def quad_tree_and_fbsp(obs_list, domain, alg):
    width = 10.0
    height = 10.0

    pp = PathPlanningProblem(width, height, 20, 3.0, 1, obs_list)
    initial = (domain.initial_pos[0]/10.0, (10.0 - (domain.initial_pos[1]/10.0)))
    goals = (domain.goal_pos[0]/10.0, (10.0 - (domain.goal_pos[1]/10.0)))

    ax = plt.subplot(111)
    ax.set_xlim(0.0, width)
    ax.set_ylim(0.0, height)
    if alg == 'QUADTREE':
        qtd = QuadTreeDecomposition(pp, 0.2)
    else:
        qtd = BinarySpacePartitioning(pp, 0.2)
    qtd.find_free_cell()
    initial_state = qtd.find_initial_state(initial, "free initial")

    goal_state = qtd.find_initial_state(goals, "free goal")
    print(alg + ' FINDING PATH USING %s ALGORITHM...')
    print(alg + ' result:' )
    start_time = time.time()
    astar(initial_state, goal_state, qtd.free_cell)
    end_time = time.time()

    print(alg +' took: ' + str(end_time - start_time) + 'seconds.')

    qtd.Draw(ax)
    n = qtd.CountCells()
    ax.set_title(alg + ' Decomposition\n{0} cells'.format(n))
    plt.show()


def draw_map(domain, obs_list, map):
    width = 10.0
    height = 10.0

    pp = PathPlanningProblem(width, height, 20, 3.0, 1, obs_list)
    initial = (domain.initial_pos[0] / 10.0, (10.0 - (domain.initial_pos[1] / 10.0)))
    goals = (domain.goal_pos[0] / 10.0, (10.0 - (domain.goal_pos[1] / 10.0)))

    ax = plt.subplot(111)
    ax.set_xlim(0.0, width)
    ax.set_ylim(0.0, height)

    qtd = QuadTreeDecomposition(pp, 0.2)
    ip = plt.Rectangle((initial[0], initial[1]), 0.1, 0.1, facecolor='#8B0000')
    ax.add_patch(ip)
    ip = plt.Rectangle((goals[0], goals[1]), 0.1, 0.1, facecolor='#228B22')
    ax.add_patch(ip)

    for i in range(len(map[0])):
        for j in range(len(map[0])):
            if map[i][j] != 'o' and map[i][j] != ' ' and map[i][j] != 'I' and map[i][j] != 'G':
                ip = plt.Rectangle((i/10.0, (10 - (j/10.0))), 0.1, 0.1, facecolor='#FFA500')
                ax.add_patch(ip)
    qtd.Draw(ax)
    plt.show()

if __name__ == '__main__':
    main()
