"""
Returns a collision free path in 2D planning domain.

1. The RRT generation algorithm can generate tree spanning 
   the free space in the planning domain, and return a path
   in the tree that can avoid obstacles of any shape and sizes.

2. The input map needs to be an image convertable to NumPy array.
"""
import numpy as np
import imageio.v3 as iio
from rrt import RRT
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection as lc
from rrt import Node


def check_collision(start, end, obs_loc):
    """
    Check whether the line connecting two nodes collides with obstacles.

    basic intuition of detecting obstacle
    1. fit a line between the two node
    
    2. use an np.arange array to generate x pos
       then use the line equation to compute y pos
    
    3. check whether the pixel nearest to that position
       is black or white
    
       black is 0, white is 1

    Input:
        start: Start position.
        end: End position.
        obs_loc: Locations of obstacle pixels.

    Output:
        check: Bool indicating if collision happens.
    """
    # when the line is vertical
    if abs(start[0] - end[0]) <= 2:
        y0 = int(start[1])
        y1 = int(end[1])
        x0 = int(start[0])

        x = x0 * np.ones((1, abs(y0 - y1)))
        min_y = np.min([y0, y1])
        max_y = np.max([y0, y1])
        y = np.arange(min_y, max_y, step=1)
        line_coord = np.vstack((x, y)).T
    else:
        # solve for the line between the two nodes
        A = np.array([[start[0], 1], [end[0], 1]])
        b = np.array([start[1], end[1]])
        root = np.linalg.solve(A, b)
        k = root[0]
        b = root[1]

        # get the (x, y) coordinates on the line
        min_x = np.min([start[0], end[0]])
        max_x = np.max([start[0], end[0]])
        x = np.arange(min_x, max_x, step=1)
        y = np.asarray(k * x + b, dtype=int)
        line_coord = np.vstack((np.asarray(x, dtype=int), y)).T

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

def check_in_obs(pt, obs_loc):
    """
    Check whether the given node is inside the obstacle.

    Input:
        pt: Target node position.
        obs_loc: Locations of the obstacles.

    Output:
        True if inside the obstacle, otherwise return False.
    """
    pt = np.copy(pt)

    # convert the point to int
    # if it's near or inside the obstacle
    # the converted point will be overlapped
    # with one of the obstacle pixels.
    pt = np.asarray(pt, dtype=int)
    dist = np.linalg.norm(obs_loc-pt, axis=1)
    zero_elem = dist[np.where(dist == 0.0)]

    return True if zero_elem.size > 0 else False


def obstacle_avoidance(obstacle_pos, q_init, k, delt, domain, goal):
    """
    Generate a random tree that avoids the obstacles.

    Input:
        obstacle_pos: Location of the obstacle pixels.
        q_init: Start position.
        k: Max number of nodes on the random tree.
        delt: Step size.
        domain: Planning domain.
        goal: Goal position.

    Output:
        All the nodes in this RRT (ordered sequentially).
    """    
    rrt = RRT(q_init, k, delt, domain)
    node_num = rrt.get_node_num()

    while node_num < k:
        
        # return if no obstacle between this node and goal
        # get pos of latest node:
        latest_node = rrt.get_latest_node()
        latest_pos = latest_node.get_pos()
        collision = check_collision(latest_pos, goal, obstacle_pos)

        if not collision:
            rrt.expand(latest_node, goal)
            break

        # if flag is true, then the next node is valid
        flag = False
        tag_node = None
        # if collision happens, expand to other vertices
        while not flag:
            # check whether new vertex is in obstacle
            # new vertex:
            parent_node, new_pos = rrt.get_new_pos(tag_node=tag_node)
            collision = check_in_obs(new_pos, obstacle_pos)

            if not collision:
                flag = True
            else:
                # if collision is true, this parent node 
                # needs to be tagged as invalid so rrt
                # will not select this node again as parent.
                tag_node = parent_node

        # if passed the above while loop
        rrt.expand(parent_node, new_pos)
        node_num = rrt.get_node_num()

    return rrt.get_node_list()

def path_finder(start: Node, end: Node):
    """
    Find the path from start to goal in the tree generated.

    Input:
        start (Node): Start pos of the path.
        end (Node): End pos of the path.

    Output:
        path (list of Nodes): Nodes for the path.
    """
    path = [end]
    parent = end.get_parent()
    parent_pos = parent.get_pos()
    start_pos = start.get_pos()
    while parent_pos[0] != start_pos[0] and parent_pos[1] != start_pos[1]:
        path.append(parent)
        parent = parent.get_parent()
        parent_pos = parent.get_pos()

    return path

def draw_lines(line_seg, start, goal, path, obs):
    """
    Visualize the path with matplotlib.

    Input:
        line_seg (NumPy array): The entire tree generated.
        start (NumPy array): Start position of the path.
        goal (NumPy array): Goal position of the path.
        path (NumPy array): Solved path from start to goal.
        obs (NumPy array): Map of obstacle.

    Output:
        None.
    """

    l_c = lc(line_seg)
    l_c_path = lc(path, color = 'r')
    _, ax = plt.subplots()

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
    """
    Specify start and goal location here and plot the path.

    Input:
        map (NumPy array): Map input as numpy array.
        obs_loc (NumPy array): Location of the obstacle pixels.

    Output:
        path (NumPy array): Numpy array with solved path.
    """

    delt = 1
    D = 100
    K = 2000

    # Determine the start and goal positions
    q_init = np.array([40, 40])
    goal = np.array([60, 60])
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

    return path_seg

def test(obs_loc, path=None):
    """
    Test the obstacle avoidance algorithm.

    Input:
        obs_loc (NumPy array): Location of the obstacle pixels.
        path (Numpy array): Solved path.

    Output:
        None
    """

    if path == None:
        # choose a point inside obstacle
        pt_in = np.array([30.5, 20.5])
        # choose a point in free space
        pt_out = np.array([40, 40])

        result = check_in_obs(pt_in, obs_loc)
        print("check in obstacle: " + str(result))

        result = check_in_obs(pt_out, obs_loc)
        print("check outside obstacle: " + str(result))

        return

    for waypts in path:
        start = waypts[0]
        end = waypts[1]

        check = check_collision(start, end, obs_loc)
        if check:
            print("there are obstacles in between!" + str(end))

            flag = check_in_obs(end, obs_loc)
            print(flag)

if __name__ == "__main__":
    map = iio.imread('N_map.png')
    map = np.flipud(map)
    
    # find indices for obs:
    map = map[1:98, :]
    map = map[:, 1:98]
    obs = np.where(map == 1)
    obs = np.vstack((obs[1], obs[0])).T

    path = graphing(map, obs)
    test(obs, path)
