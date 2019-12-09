'''
Filename: coverage_path.py
Description: HW 5: Module handling planning the path for optimal coverage using spanning trees.
Author: Andi Frank
E-mail: a2frank@eng.ucsd.edu
Purpose: CSE 276A - Intro to Robotics; Fall 2019
'''

##############################################################
#                       IMPORTS
##############################################################

import numpy as np
import scipy as sp
import matplotlib as mpl
import matplotlib.pyplot as plt
import networkx as nx
import copy
from math import pi, sin, cos, tan, atan, atan2



#########################################################################################

# Credit to Stack Overflow users Gareth Rees for the intersection proof below and and 
#   Ibraim Ganiev for the implementation in python.
# See: https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect


def np_cross_product(a, b):
    return a[0]*b[1]-a[1]*b[0]

def np_seg_intersect(a, b, considerCollinearOverlapAsIntersect = False):
    # https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect/565282#565282
    # http://www.codeproject.com/Tips/862988/Find-the-intersection-point-of-two-line-segments
    
    r = a[1] - a[0] # vector representation of segment A
    s = b[1] - b[0] # vector representation of segment B
    v = b[0] - a[0]
    # Calculate 2D cross products
    num = np_cross_product(v, r)
    denom = np_cross_product(r, s)
#     # If r x s = 0 and (q - p) x r = 0, then the two lines are collinear. --> Never the case in this implementation
#     if np.isclose(denom, 0) and np.isclose(num, 0):
#         return None
    if np.isclose(denom, 0) and not np.isclose(num, 0):
        # Parallel and non intersecting
        return None
    u = num / denom
    t = np_cross_product(v, s) / denom
    if u >= 0 and u <= 1 and t >= 0 and t <= 1:
        res = b[0] + (s*u)
        return res
    # Otherwise, the two line segments are not parallel but do not intersect.
    return None

###############################################################################################


def spanning_tree(grid,pos):
    '''
    Create a spanning tree representation of the connectivity of an NxM grid.
    '''
    # Step 3: Find minimum spanning tree
    tree = nx.minimum_spanning_tree(grid)
    nx.relabel_nodes(tree,pos,False)

    return tree


def get_valid_moves(grid, tree):
    '''
    Make a graph of all valid movements between cells based on Spanning Trees.
    Grid edges should not pass through spanning tree edges.
    '''
    G = copy.deepcopy(grid.edges)
    for edge in G:
        for path in tree.edges:
            # Intersection test
            if np_seg_intersect(np.array(edge), np.array(path)) is not None:
                grid.remove_edge(*edge)
    
    return grid

def create_grid(rows,cols,resolution):
    '''
    Create 2D grid with positions.
    '''
    grid_lims  = np.array([[0,rows], [0,cols]]) # [m]
    resolution = 1 # [m]
    rmax, cmax = ((grid_lims[:,1]-grid_lims[:,0])/resolution).astype(int)

    grid = nx.grid_2d_graph(rmax,cmax)
    for i,j in np.ndindex(rmax,cmax):
        grid.nodes[(i,j)]["pos"] = (i,cmax-1-j)

def get_macro_graph(grid):
    grid_lims = np.array( [ [0,grid.shape[0]], [0, grid.shape[1]] ] )
    ROWMAX, COLUMNMAX = ((grid_lims[:,1]-grid_lims[:,0])/2).astype(int)
    macro_graph = nx.grid_2d_graph(ROWMAX,COLUMNMAX)

    pos = dict( ( (i,j), (2*i+1/2, 2*(COLUMNMAX-1-j)+1/2) ) for
                (i,j) in np.ndindex(ROWMAX, COLUMNMAX))
    for (i,j) in np.ndindex(ROWMAX, COLUMNMAX):
        this_pos = pos[i,j]
        macro_graph.nodes[(i,j)]["pos"] = this_pos

    return macro_graph



def done(neighbors, path):
    for node in neighbors:
        # If we find any neighbor not in the path, you can move forward.
        if node not in path.nodes:
            return False
    # If all neighbors are already in path, you're done!
    return True

def best_move(my_pos, my_heading, path, grid, bias=pi/3):
    neighbors = list(grid.neighbors(my_pos))
    # Check if we're out of valid neighbors
    if done(neighbors, path):
        return None
    # Otherwise find best choice
    direction = []
    cost = []
    for node in neighbors:
        # We want to ignore nodes that we've already passed
        if node in path.nodes:
            direction.append(float('inf'))
            cost.append(float('inf'))
        else:
            diff = np.array(node) - np.array(my_pos)
            direction.append(atan2(diff[1],diff[0]))
            cost.append( abs(direction[-1]-(my_heading+bias)) )
            if cost[-1]<0:
                cost[-1] += 2*pi # consider left turns only --> like walking with your hand on the left wall
    choice = np.argmin(cost)
    goal_pos = neighbors[choice]
    goal_heading = direction[choice]


def calculate_path(valid_moves):
    path = nx.Graph()
    path.add_node((0,0), pos=(0,0))

    my_pos = (0,0)
    my_heading = 0
    bias = pi/3

    # Negative bias for first step -- first find the wall, then bias left turns
    move = best_move(my_pos, my_heading, path, bias=-bias)
    if move is not None:
        goal_pos, goal_heading = move
    path.add_node(goal_pos, pos=goal_pos)
    path.add_edge(my_pos, goal_pos)
    my_pos = goal_pos
    my_heading = goal_heading

    # Keep hand on same side of "wall"
    neighbors = list(valid_moves.neighbors(my_pos))
    while( True ):
        move = best_move(my_pos, my_heading, path, bias=bias)
        if move is None:
            break
        else:
            goal_pos, goal_heading = move
        
        # picar.to_waypoint(goal)
        path.add_node(goal_pos, pos=goal_pos)
        path.add_edge(my_pos, goal_pos)
        my_pos = goal_pos
        my_heading = goal_heading

    return path
    
    
    
def get_path(N,M,resolution=1):
    # Step 1: Define grid
    grid = create_grid(N,M,resolution)

    # Step 2: Define macro-grid -- one macro grid square holds a 2x2 base grid squares
            # Define nodes at the center of macro grid squares
    macro = get_macro_graph(grid)

    # Step 3: Get spanning tree of macro-grid
    tree = spanning_tree(macro_grid)

    # Step 4: Use macro-grid to guide path
    valid_moves = get_valid_moves(grid, tree)
    path = get_path(valid_moves)
    


