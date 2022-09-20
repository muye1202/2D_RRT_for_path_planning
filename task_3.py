import numpy as np
import imageio.v3 as iio
from rrt import Node, RRT
from obstacle_avoidance import draw_lines, path_finder
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
    # when the line is vertical
    if abs(start[0] - end[0]) <= 1:
        y0 = int(start[1])
        y1 = int(end[1])
        x0 = int(start[0])
        x1 = int(end[0])

        x = x0 * np.ones((1, abs(y0 - y1)))
        min_y = np.min([y0, y1])
        max_y = np.max([y0, y1])
        y = np.arange(min_y, max_y, step=1)
        line_coord = np.vstack((x, y)).T
    else:
        # solve for the line between the two nodes
        def center_line(sol):
            return [sol[0] * start[0] + sol[1] - start[1], sol[0] * end[0] + sol[1] - end[1]]
        root = fsolve(center_line, [start[0], start[1]])
        k = root[0]
        b = root[1]

        # get the (x, y) coordinates on the line
        min_x = np.min([start[0], end[0]])
        max_x = np.max([start[0], end[0]])
        x = np.arange(min_x, max_x, step=1)
        y = np.asarray(k * x + b, dtype=int)
        line_coord = np.vstack((x, y)).T

    ## obs_pos matrix - pt_pos_matrix
    ## if there're zero values, it means the two points are the same  
    count = 0
    for i in range(len(line_coord)):
        pt = line_coord[i].reshape((1,2))
        dist = np.linalg.norm(obs_loc-pt, axis=1)
        zero_elem = dist[np.where(dist == 0.0)]

        if zero_elem.size > 0:
            count += 1

    if count == 0:
        check = False
    else:
        check = True
    
    return check

def obstacle_avoidance(obstacle_pos: list, q_init, k, delt, domain, goal):
    rrt = RRT(q_init, k, delt, domain)
    node_num = rrt.get_node_num()

    #exist = False
    while node_num < k:
        
        # return if no obstacle between this node and goal
        # get pos of latest node:
        latest_node = rrt.get_latest_node()
        latest_pos = latest_node.get_pos()
        latest_pos = np.asarray(latest_pos, dtype=int)
        collision = check_collision(latest_pos, goal, obstacle_pos)

        if collision == False:
            # print('it will stop here')
            rrt.expand(latest_node, goal)
            break

        flag = False
        # if collision happens, expand to other vertices
        while flag == False:
            # check whether new vertex is in obstacle
            # new vertex:
            nearest_node, new_pos = rrt.get_new_pos()
            curr_pos = nearest_node.get_pos()
            curr_pos = np.asarray(curr_pos, dtype=int)
            collision = check_collision(curr_pos, new_pos, obstacle_pos)

            if collision == False:
                flag = True


        # if passed the above while loop
        rrt.expand(nearest_node, new_pos)
        node_num = rrt.get_node_num()

    return rrt.get_node_list()

def draw_lines(line_seg, start, goal, path, obs):
    l_c = lc(line_seg)
    l_c_path = lc(path, color = 'r')
    f, ax = plt.subplots()

    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.add_collection(l_c, '-o')
    ax.add_collection(l_c_path, '-o')
    
    plt.plot(start[0], start[1], 'ro')
    plt.plot(goal[0], goal[1], 'go')
    
    # plot the map
    plt.imshow(obs, origin='lower', cmap='gray')
    plt.show()

def graphing(map, obs_loc):

    delt = 1
    D = 100
    K = 600
    q_init = np.array([10, 5])
    goal = np.array([38, 37])
    node_list = obstacle_avoidance(obs_loc, q_init, K, delt, D, goal)
    nodes_pos_x = []
    nodes_pos_y = []
    line_seg = []

    for node in node_list:
        pos = node.get_pos()
        parent_node_pos = (pos[0], pos[1])

        nodes_pos_x.append(pos[0])
        nodes_pos_y.append(pos[1])

        # get a line collection
        for child in node.get_child():
            c_pos = child.get_pos()
            child_pos = (c_pos[0], c_pos[1])
            seg = [parent_node_pos, child_pos]
            line_seg.append(seg)

    # get path collection:
    path = path_finder(node_list[0], node_list[len(node_list)-1])
    path_seg = []
    for i in range(len(path)-1):
        curr_node = path[i]
        cpos = curr_node.get_pos()
        curr_pos = (cpos[0], cpos[1])

        next_node = path[i+1]
        npos = next_node.get_pos()
        next_pos = (npos[0], npos[1])
        seg_path = [curr_pos, next_pos]
        path_seg.append(seg_path)

    draw_lines(line_seg, start=q_init, goal=goal, path=path_seg, obs=map)

def test(map, obs_loc):

    start = np.array([0, 0])
    end = np.array([80, 80])

    check = check_collision(start, end, obs_loc)
    print("there're obstacles in between: ")
    print(check)

    plt.imshow(map)
    plt.plot(start[0], start[1], 'go')
    plt.plot(end[0], end[1], 'ro')
    plt.show()

if __name__ == "__main__":
    map = iio.imread('N_map.png')
    map = np.flipud(map)
    
    # find indices for obs:
    map = map[1:98, :]
    map = map[:, 1:98]
    obs = np.where(map == 1)
    obs = np.vstack((obs[0], obs[1])).T
    obs_loc = np.zeros(obs.shape)
    obs_loc[:, 0] = obs[:, 1]
    obs_loc[:, 1] = obs[:, 0]

    # print("obstacle looks like: ")
    # plt.plot(obs_loc[:, 0], obs_loc[:, 1], 'o')
    # plt.show()
    # exit()

    #test(map, obs_loc)
    graphing(map, obs_loc)

    
    
