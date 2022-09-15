import numpy as np
import imageio.v3 as iio
from rrt import Node, RRT
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection as lc
from scipy.optimize import fsolve

# basic intuition of detecting obstacle
# 1. fit a line between the two node
#
# 2. use an np.arange array to generate x pos
#    then use the line equation to compute y pos
#
# 3. check whether the pixel nearest to that position
#    is black or white
#
#    black is 0, white is 1

def check_collision(start, end, obs_loc):
    check = False

    # solve for the line between the two nodes
    def center_line(sol):
        return [sol[0] * start[0] + sol[1] - start[1], sol[0] * end[0] + sol[1] - end[1]]
    root = fsolve(center_line, [start[0], end[0]])
    k = root[0]
    b = root[1]

    # get the (x, y) coordinates on the line
    x = np.arange(start[0], end[0], step=1)
    y = np.asarray(k * x + b, dtype=int)
    line_coord = np.vstack((x, y)).T

    # check whether the point overlap with obstacles
    collision = 0

    ## vectorize the process
    ## obs_pos matrix - pt_pos_matrix
    ## if there're zero values, it means the two points are the same  
    num_of_row = obs_loc.shape[0]
    num_of_col = obs_loc.shape[1]
    num_of_pts_on_line = line_coord.shape[0]
    x_rep_matrix = np.repeat(x, num_of_col)
    x_coord = np.zeros((num_of_row, num_of_pts_on_line*num_of_col)) + x_rep_matrix.reshape((1, -1))
    y_rep_matrix = np.repeat(y, num_of_col)
    y_coord = np.zeros((num_of_row, num_of_pts_on_line*num_of_col)) + y_rep_matrix
    # tile the obs pos matrix:
    obs_pos_mat = np.tile(obs_loc, num_of_pts_on_line)
    col = obs_pos_mat.shape[1]
    row = obs_pos_mat.shape[0]

    xy_coord = np.zeros(obs_pos_mat.shape)
    xy_coord[:, 0:col-1:1] = x_coord[:, 0:col-1:1]
    xy_coord[:, 1:col:1] = y_coord[:, 1:col:1]
    
    # calculate difference:
    difference = xy_coord - obs_pos_mat
    zero_elem = difference[np.where(difference == 0)]

    # if zero_elem has zero elements, it means there're overlapping points
    check = zero_elem.size > 0

    return check

def test():
    map = np.array([
        [0, 0, 0, 0],
        [0, 1, 1, 0],
        [0, 1, 1, 0],
        [0, 0, 0, 0]
    ])

    start = np.array([0, 0])
    end = np.array([3, 0])
    obs_loc = np.where(map == 1)
    obs_loc = np.vstack((obs_loc[0], obs_loc[1])).T

    check = check_collision(start, end, obs_loc)
    print("there're obstacles in between: ")
    print(check)

if __name__ == "__main__":
    test()
    exit()

    map = iio.imread('N_map.png')
    
    # find indices for obs:
    obs_loc = np.where(map == 1)
    obs_loc = np.vstack((obs_loc[0], obs_loc[1])).T
    start = np.array([0, 2])
    end = np.array([10, 10])
    # check = check_collision(start, end, obs_loc)
    
    
    
