'''
Filename: costmap.py
Description: My implementation of a costmap representation.
Author: Andi Frank
E-mail: a2frank@eng.ucsd.edu
Purpose: CSE 276A - Intro to Robotics; Fall 2019
'''

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
from scipy import ndimage as ndi
from skimage.morphology import medial_axis
from skimage.draw import line
from math import sqrt
from copy import copy, deepcopy
import itertools as it



class Rect():
    def __init__(self, width, height, anchor, anchor_type='origin', **kwargs):
        anchor = np.array(anchor)
        self.width  = width
        self.height = height
        if anchor_type  == 'origin':
            self.left   = anchor[0]
            self.bottom = anchor[1]
            self.center = anchor + np.array([self.width, self.height])/2
        elif anchor_type == 'center':
            self.center = anchor
            self.left   = self.center[0] - self.width/2
            self.bottom = self.center[1] - self.height/2
        else:
            raise Exception('anchor_type argument can only accept values'
                            '\'origin\' or \'center\' (\'origin\' by default)')
            
        self.right  = self.left + width
        self.top    = self.bottom + height
        self.corners =  np.array([ [self.left,  self.bottom], 
                                   [self.left,  self.top   ], 
                                   [self.right, self.top   ], 
                                   [self.right, self.bottom]
                            ])
        self.opts = kwargs



# @TODO: MOVE ALL VORONOI/PATH PLANNING STUFF TO A DIFFERENT CLASS THAT 
#        TAKES A MAP AS AN ARGUMENT

class Map():
    '''
    Costmap represented as a grid array. Handles translating from (x,y) coordinate space
    to (row,col) indexing of arrays.
    '''

    def __init__(self, origin, xlim, ylim, resolution=1, fill=0):
        '''
        NOTE: Map uses (x,y) indexing, not (row,col) indexing like np.ndarrays
        '''
        self.origin = np.array(origin, dtype=float)
        self.xlim = np.array(xlim, dtype=float)
        self.ylim = np.array(ylim, dtype=float)
        self.res = resolution
        
        width  = int(np.diff(xlim)/self.res) + 1 # e.g. if lim = [5,6],  we want
        height = int(np.diff(ylim)/self.res) + 1 # 2 slots (5,6) not just 1
        
        # Grid set up so nrows=height, ncols=width
        self.grid = np.full([height, width], fill)
    
    def _to_ind(self, coord, xy='xy', dtype=None):
        coord = np.array(coord)
        while np.ndim(coord)<2:
            coord = coord[None]
        if   xy=='x':
            # Translate origin to [0,0]
            coord = coord - self.origin[0]
        elif xy=='y':
            # Translate origin to [0,0]
            coord = coord - self.origin[1]
        elif xy=='xy':
            # Translate origin to [0,0]
            coord = coord - self.origin
            # Switch order of coords: matrix indexes (vert,horz), not (horz,vert)
            coord = np.roll(coord, 1, axis=1)
        else:
            raise Exception('xy parameter can only accept inputs \'x\', \'y\', or \'xy\'.') 
         # Scale by resolution and cast to integers.
        ind = (coord/self.res).astype(int) 
        if 'y' in xy:
            # Invert rows axis -> coord (0,0) is at bottom left, grid (0,0) is at top left
            grid_top = self.grid.shape[0]-1
            ind[:,0] = grid_top - ind[:,0]
        # Remove any added unnecessary dimensions
        ind = np.squeeze(ind).astype(int)
        # Cast to desired datatype. Or, return as list of arrays if input as list.
        if dtype is not None:
            ind = dtype(coord)
        return ind

    
    def rect_to_inds(self, rect):
        left   = self._to_ind(rect.left,   xy='x')
        right  = self._to_ind(rect.right,  xy='x')
        bottom = self._to_ind(rect.bottom, xy='y')
        top    = self._to_ind(rect.top,    xy='y')
        shape = np.zeros([bottom-top+1, right-left+1, 2])
        i=0
        for row in range(top, bottom+1):
            j=0
            for col in range(left, right+1):
                inds[i,j,:] = [row,col]
                j+=1
            i+=1
        return inds
    
    def get_rect_points(self, rect):
        left   = self._to_ind(rect.left,   xy='x')
        right  = self._to_ind(rect.right,  xy='x')
        bottom = self._to_ind(rect.bottom, xy='y')
        top    = self._to_ind(rect.top,    xy='y')
        r = np.arange(left, right+1)
        c = np.arange(top, bottom+1)
        inds = np.array([ i for i in it.product(r,c)])
        return inds
    
    def draw_points(self, pts, value=1, units='world'):
        if units is not 'grid':
            pts = self._to_ind(pts)
        if np.ndim(pts)==1:
            self.grid[pts[0],pts[1]] = value
        else:
            self.grid[pts[...,0],pts[...,1]] = value
        return self.grid
    
    def fill_rect(self, rect, value=1):
        left   = self._to_ind(rect.left,   xy='x')
        right  = self._to_ind(rect.right,  xy='x')
        bottom = self._to_ind(rect.bottom, xy='y')
        top    = self._to_ind(rect.top,    xy='y')

        for row in range(top, bottom+1):
            for col in range(left, right+1):
                self.grid[row,col] = value
                
        return self.grid

    def draw_line(self,p,q,value=1,units='world'):
        if units is not 'grid':
            p,q = self._to_ind((p,q))
        r,c = line(*p,*q)
        self.grid[r,c] = value
        return self.grid

    def outline_rect(self, rect, value=1):
        pts = rect.corners
        for _ in range(len(pts)):
            pts = np.roll(pts,1,axis=0)
            self.draw_line(pts[0],pts[1])
        return self.grid
    
    
    def voronoi(self):
        return medial_axis(1-self.grid)
    
    def add_voronoi(self):
        '''
        Note: Voronoi is added with value -1 to distinguish from obstacles with value 1
        '''
        self.grid -= self.voronoi()
        return self.grid
    
    def find_nearest(self, pt, grid=None, value=1, units='world'):
        if units is not 'grid':
            ind = self._to_ind(pt)
        else:
            ind = pt
        if grid is None:
            grid = self.grid
        # Find nearest point on map
        path_inds = np.argwhere(grid==value)
        distances = np.power( np.sum(  np.power( (path_inds-ind), 2), axis=1), 0.5)
        ind2 = path_inds[ np.argmin(distances) ]
        return ind2
        
    
    def get_to_path(self, pt, path_mask, units='world'):
        if units is not 'grid':
            ind = self._to_ind(pt)
        else:
            ind = pt
        ind2 = find_nearest(pt=ind, grid=path_mask, units='grid')
        self.draw_line(ind, ind2, units='grid')
        return self.grid
    
    def get_to_path_biased(self, pt, path_mask, bias_map, units='world'):
        if units is not 'grid':
            ind = self._to_ind(pt)
        # Find nearest point on map
        path_inds = np.argwhere(path_mask==1)
        distances = np.power( np.sum(  np.power( (path_inds-ind), 2), axis=1), 0.5)
        biases = bias_map[path_inds[:,0],path_inds[:,1]]
        ind2 = path_inds[ np.argmin(distances+biases) ]
        self.draw_line(ind, ind2, units='grid')
        return self.grid
    
    def generate_potential_field( self, shape, 
                                  atr_inds=None, rep_inds=None, 
                                  atr_factor=1, rep_factor=1, 
                                  atr_exp=-1,    rep_exp=-1   ):
        grid = np.zeros(shape)
        if atr_inds is not None:
            grid -= atr_factor * _potential_field(pts=atr_inds, 
                                                  arr=None, arrshape=shape, 
                                                  func=exp_dist, func_args=atr_exp)
        if rep_inds is not None:
            grid += rep_factor * _potential_field(pts=rep_inds, 
                                                  arr=None, arrshape=shape, 
                                                  func=exp_dist, func_args=rep_exp)
        return grid
        
        
    


#####################################################################
#################### HELPER FUNCTIONS ###############################
#####################################################################

def normalize(arr):
    '''
    Normalize by array's max value.
    '''
    return arr / np.nanmax(np.abs(arr))

def arr_dist(pt, inds):
    '''
    Returns Euclidean distance within an array as if each element 
    were a block of size 1x1.
    '''
    return np.sqrt( np.sum( ((pt-inds) ** 2), axis=-1) ) 

def exp_dist(pt, inds, exp=1):
    '''
    Returns distance raised to the power exp.
    '''
    d = arr_dist(pt, inds)
    if exp<0:
        # Avoid dividing by zero
        zero_inds = np.argwhere( d<1 )
        d[zero_inds] = 0.5
    return np.power(d,exp)

def dist2d(p,q):
    '''
    Computes row-wise distance between list of points p and list of points q
        or a list of points and a single point.
    Dimensionality of the coordinate space is assumed to be 2d
    '''
    p = np.array(p)
    q = np.array(q)
    while np.ndim(p)<2: p = p[None]
    while np.ndim(p)<2: p = p[None]
    coord_diffs = np.sum((p,-q),axis=-1)   
    return np.sqrt( np.sum(coord_diffs ** 2) )

def _preprocess_pf(pts, arr, arrshape):
    '''
    Handle preprocessing for _potential_field()
    '''
    # Preprocess pts into array of dim <=2
    # so we can index through goal ind pairs
    pts = np.array(pts)
    while np.ndim(pts)<2:  pts=pts[None]
        
    # Preprocess input arr or create an empty arr 
    # of shape arrshape
    if arr is None:
        # Make baseline arr of all zeros
        arr = np.zeros(arrshape)
    else:
        # Make changes to a copy of the arr
        arr = copy(arr)
    # Get array of all indices of arr
    inds = np.argwhere(arr+True)
    return pts, arr, inds

def _potential_field(pts, arr=None, arrshape=None, func=arr_dist, func_args=None):
    '''
    Iterate through pts, apply func to all points on array. Sum
    affects of each pt.
    '''
    pts, arr, arr_inds = _preprocess_pf(pts, arr, arrshape)
    for pt in pts:
        if func_args is not None:
            f = func(pt, arr_inds, func_args)
        else:
            f = func(pt, arr_inds)
        arr[arr_inds[:,0], arr_inds[:,1]] += f
    return arr
        

