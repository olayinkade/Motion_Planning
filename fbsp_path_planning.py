import sys
import matplotlib.pyplot as plt
import numpy as np
import math
import copy
from pathplanning import PathPlanningProblem, Rectangle


class CellDecomposition:
    def __init__(self, domain, minimumSize):
        self.domain = domain
        self.minimumSize = minimumSize
        self.root = [Rectangle(0.0, 0.0, domain.width, domain.height), 'unknown', []]
        self.free_cell =[]
        a = 1

    def Draw(self, ax, node = None):
            if (node == None ):
                node = self.root
            r = plt.Rectangle((node[0].x, node[0].y), node[0].width, node[0].height, fill=False, facecolor=None, alpha=0.5)
            if (node[1] == 'mixed' ):
                color = '#5080ff'
                if (node[2] == [] ):
                    r.set_fill(True)
                    r.set_facecolor(color)
            elif ( node[1] == 'free' ):
                color = '#FFFFFF'
                r.set_fill(True)
                r.set_facecolor(color)
            elif ( node[1] == 'obstacle'):
                color = '#904209'
                r.set_fill(True)
                r.set_facecolor(color)
            elif node[1] == 'free initial':
                color = '#FF0000'
                r.set_fill(True)
                r.set_facecolor(color)
            elif node[1] == 'free goal':
                color = '#008000'
                r.set_fill(True)
                r.set_facecolor(color)
            elif node[1] == 'free path':
                color = '#FFA500'
                r.set_fill(True)
                r.set_facecolor(color)
            else:
                print("Error: don't know how to draw cell of type", node[1])
            #print('Draw node', node)
            ax.add_patch(r)
            for c in node[2]:
                self.Draw(ax, c)

    def find_initial_state(self, initial, curr_string):
        found = False
        for cell in self.free_cell:
                top_right = (cell[0].x + cell[0].width, cell[0].y + cell[0].height)
                if find_point(cell[0].x, cell[0].y, top_right[0], top_right[1], initial[0], initial[1]):
                    found = True
                    curr = cell
        if not found:
            curr = self.free_cell[0]
            distance = math.sqrt(((initial[0]-curr[0].x)**2)+((initial[1]-curr[0].y)**2))
            for i in range(len(self.free_cell)-1):
                new_dist = math.sqrt(((initial[0]-self.free_cell[i+1][0].x)**2)+((initial[1]-self.free_cell[i+1][0].y)**2))
                if new_dist < distance:
                    distance = new_dist
                    curr = self.free_cell[i+1]

        curr[1] = curr_string
        return curr


    def find_free_cell(self, node=None):
        if node is None:
            node = self.root
            r = plt.Rectangle((node[0].x, node[0].y), node[0].width, node[0].height, fill=False, facecolor=None, alpha=0.5)
        if node[1] == 'free':
            self.free_cell.append(node)
        for c in node[2]:
            self.find_free_cell(c)

    def CountCells(self, node = None ):
        if ( node is None ):
            node = self.root
        sum = 0
        if ( node[2] != [] ):
            sum = 0
            for c in node[2]:
                sum = sum + self.CountCells(c)
        else:
            sum = 1
        return sum


class QuadTreeDecomposition(CellDecomposition):
    def __init__(self, domain, minimumSize):
        super().__init__(domain, minimumSize)
        self.root = self.Decompose(self.root)

    def Decompose(self, node):
        cell = 'free'
        r = node[0]
        rx = r.x
        ry = r.y
        rwidth = r.width
        rheight = r.height

        for o in self.domain.obstacles:
            if ( o.CalculateOverlap(r) >= rwidth * rheight ):
                cell = 'obstacle'
                break
            elif ( o.CalculateOverlap(r) > 0.0 ):
                cell = 'mixed'
                break
        if ( cell == 'mixed'):
            if (rwidth / 2.0 > self.minimumSize) and (rheight / 2.0 > self.minimumSize):
                childt1 = [Rectangle(rx, ry, rwidth/2.0, rheight/2.0), 'unknown', [] ]
                qchild1 = self.Decompose( childt1 )
                childt2 = [Rectangle(rx + rwidth/2.0, ry, rwidth/2.0, rheight/2.0), 'unknown', [] ]
                qchild2 = self.Decompose( childt2 )
                childt3 = [Rectangle(rx, ry + rheight/2.0, rwidth/2.0, rheight/2.0), 'unknown', [] ]
                qchild3 = self.Decompose( childt3 )
                childt4 = [Rectangle(rx + rwidth/2.0, ry + rheight/2.0, rwidth/2.0, rheight/2.0), 'unknown', [] ]
                qchild4 = self.Decompose( childt4 )
                children = [ qchild1, qchild2, qchild3, qchild4 ]
                node[2] = children
            else:
                 cell = 'obstacle'
        node[1] = cell
        return node


class BinarySpacePartitioning(CellDecomposition):
    def __init__(self, domain, minimumSize ):
        super().__init__(domain, minimumSize)
        self.root = self.Decompose(self.root)

    def Entropy(self, p):
        e = 0.0
        if ( ( p > 0 ) and ( p < 1.0 ) ):
            e = -p * math.log(p,2) - (1-p) * math.log(1-p,2)
        return e

    def CalcEntropy(self, rect):
        area = rect.width * rect.height
        a = 0.0
        for o in self.domain.obstacles:
            a = a + rect.CalculateOverlap(o)
        p = a / area
        return self.Entropy(p)

    def Decompose(self, node):
        cell = 'free'
        r = node[0]
        rx = r.x
        ry = r.y
        rwidth = r.width
        rheight = r.height
        area = rwidth * rheight

        for o in self.domain.obstacles:
            if ( o.CalculateOverlap(r) >= rwidth * rheight ):
                cell = 'obstacle'
                break
            elif ( o.CalculateOverlap(r) > 0.0 ):
                cell = 'mixed'
                break

        if ( cell == 'mixed'):
            entropy = self.CalcEntropy(r)
            igH = 0.0
            hSplitTop = None
            hSplitBottom = None
            vSplitLeft = None
            vSplitRight = None
            if ( r.height / 2.0 > self.minimumSize):
                hSplitTop = Rectangle(rx, ry + rheight/2.0, rwidth, rheight/2.0)
                entHSplitTop = self.CalcEntropy(hSplitTop)
                hSplitBottom = Rectangle(rx, ry, rwidth, rheight/2.0)
                entHSplitBottom = self.CalcEntropy( hSplitBottom )

                igH = entropy - ( r.width * r.height / 2.0 ) / area * entHSplitTop \
                      - ( r.width * r.height / 2.0 ) / area * entHSplitBottom
            igV = 0.0
            if ( r.width / 2.0 > self.minimumSize ):
                vSplitLeft = Rectangle(rx, ry, rwidth/2.0, rheight )
                entVSplitLeft = self.CalcEntropy( vSplitLeft )
                vSplitRight = Rectangle( rx + rwidth/2.0, ry, rwidth/2.0, rheight)
                entVSplitRight = self.CalcEntropy( vSplitRight)
                igV = entropy - ( r.width/2.0 * r.height ) / area * entVSplitLeft \
                      - ( r.width/2.0 * r.height ) / area * entVSplitRight
            children = []
            if ( igH > igV ):
                if ( igH > 0.0 ):
                    if ( hSplitTop is not None ) and ( hSplitBottom is not None ):
                        childTop = [ hSplitTop, 'unknown', [] ]
                        childBottom = [hSplitBottom, 'unknown', [] ]
                        children = [ childTop, childBottom]
            else:
                if ( igV > 0.0 ):
                    if ( vSplitLeft is not None ) and ( vSplitRight is not None ):
                        childLeft = [vSplitLeft, 'unknown', [] ]
                        childRight = [ vSplitRight, 'unknown', [] ]
                        children = [ childLeft, childRight ]
            for c in children:
                self.Decompose(c)
            node[2] = children
        node[1] = cell
        return node


class node():
    def __init__(self, parent=None, cell=None):
        self.parent = parent
        self.cell = cell

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.cell == other.cell


def generate_neigbours(curr, free_cells):
    width_2 = curr[0].width/2
    height_2 = curr[0].height/2
    cx = curr[0].x + width_2
    cy = curr[0].y + height_2
    pcx = cx + width_2 + 0.1
    ncx = curr[0].x - 0.1
    pcy = cy + height_2 + 0.1
    ncy = curr[0].y - 0.1
    neighbour = []

    for cell in free_cells:
        if curr != cell:
            top_right = (cell[0].x + cell[0].width, cell[0].y + cell[0].height)
            if find_point(cell[0].x, cell[0].y, top_right[0], top_right[1], pcx, cy + 0.1):
                if cell not in neighbour:
                    neighbour.append(cell)
            if find_point(cell[0].x, cell[0].y, top_right[0], top_right[1], pcx, pcy):
                if cell not in neighbour:
                    neighbour.append(cell)
            if find_point(cell[0].x, cell[0].y, top_right[0], top_right[1], pcx, ncy-0.1):
                if cell not in neighbour:
                    neighbour.append(cell)
            if find_point(cell[0].x, cell[0].y, top_right[0], top_right[1], ncx, cy + 0.1):
                if cell not in neighbour:
                    neighbour.append(cell)
            if find_point(cell[0].x, cell[0].y, top_right[0], top_right[1], ncx, pcy):
                if cell not in neighbour:
                    neighbour.append(cell)
            if find_point(cell[0].x, cell[0].y, top_right[0], top_right[1], ncx, ncy -0.1):
                if cell not in neighbour:
                    neighbour.append(cell)
            if find_point(cell[0].x, cell[0].y, top_right[0], top_right[1], cx -0.1, pcy):
                if cell not in neighbour:
                    neighbour.append(cell)
            if find_point(cell[0].x, cell[0].y, top_right[0], top_right[1], cx -0.1, ncy):
                if cell not in neighbour:
                    neighbour.append(cell)

            if find_point(cell[0].x, cell[0].y, top_right[0], top_right[1], pcx, cy - 0.1):
                if cell not in neighbour:
                    neighbour.append(cell)
            if find_point(cell[0].x, cell[0].y, top_right[0], top_right[1], pcx, pcy-0.2):
                if cell not in neighbour:
                    neighbour.append(cell)
            if find_point(cell[0].x, cell[0].y, top_right[0], top_right[1], pcx, ncy+0.1):
                if cell not in neighbour:
                    neighbour.append(cell)
            if find_point(cell[0].x, cell[0].y, top_right[0], top_right[1], ncx, cy -0.1):
                if cell not in neighbour:
                    neighbour.append(cell)
            if find_point(cell[0].x, cell[0].y, top_right[0], top_right[1], ncx, pcy - 0.02):
                if cell not in neighbour:
                    neighbour.append(cell)
            if find_point(cell[0].x, cell[0].y, top_right[0], top_right[1], ncx, ncy + 0.1):
                if cell not in neighbour:
                    neighbour.append(cell)
            if find_point(cell[0].x, cell[0].y, top_right[0], top_right[1], cx+0.1, pcy):
                if cell not in neighbour:
                    neighbour.append(cell)
            if find_point(cell[0].x, cell[0].y, top_right[0], top_right[1], cx + 0.1, ncy):
                if cell not in neighbour:
                    neighbour.append(cell)

    return neighbour


def find_point(x1, y1, x2, y2, x, y):
    if x > x1 and x < x2 and y > y1 and y < y2 :
        return True
    else:
        return False


def astar(initial, goal, free_cells):

    start_node = node(None, initial)
    start_node.g = start_node.h = start_node.f = 0
    end_node = node(None, goal)
    end_node.g = end_node.h = end_node.f = 0

    open_list = []
    closed_list = []
    open_list.append(start_node)
    while len(open_list) > 0:
        curr = open_list[0]
        curr_index = 0
        for index, item in enumerate(open_list):
            if item.f < curr.f:
                curr = item
                curr_index = index

        open_list.pop(curr_index)
        closed_list.append(curr)

        children = []
        temp_child = generate_neigbours(curr.cell, free_cells)
        for item in temp_child:
            children.append(node(curr, item))

        if curr == end_node or end_node in children:
            path = []
            current = curr
            while current is not None:
                path.append(current.cell)
                current = current.parent
            for cell in path:
                if cell[1] == 'free':
                    cell[1] = 'free path'
            break

        for child in children:
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            child.g = curr.g + 1
            child.h = math.sqrt(((child.cell[0].x - goal[0].x)**2)+((child.cell[0].y-goal[0].y)**2))
            child.f = child.g + child.h

            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            open_list.append(child)


