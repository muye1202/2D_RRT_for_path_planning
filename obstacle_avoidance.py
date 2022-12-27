from rrt import RRT, Node
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection as lc
from scipy.optimize import fsolve

def obstacle_avoidance(obstacle_pos: list, obstacle_r: list, q_init, k, delt, domain, goal):
    rrt = RRT(q_init, k, delt, domain)
    node_num = rrt.get_node_num()

    #exist = False
    while node_num < k:
        
        # return if no obstacle between this node and goal
        collision = 0
        # get pos of latest node:
        latest_node = rrt.get_latest_node()
        latest_pos = latest_node.get_pos()
        for i in range(len(obstacle_pos)):
            inter = check_intersection(latest_pos, goal, obstacle_pos[i], obstacle_r[i])
            if inter:
                collision += 1

        if collision == 0:
            rrt.expand(latest_node, goal)
            break

        flag = False
        # if collision happens, expand to other vertices
        while flag == False:
            # check whether new vertex is in obstacle
            collision = 0
            # new vertex:
            nearest_node, new_pos = rrt.get_new_pos()
            curr_pos = nearest_node.get_pos()
            for i in range(len(obstacle_pos)):
                check = check_intersection(new_pos, curr_pos, obstacle_pos[i], obstacle_r[i])
                if check == True:
                    collision += 1

            if collision == 0:
                flag = True

        # if passed the above while loop
        rrt.expand(nearest_node, new_pos)
        node_num = rrt.get_node_num()

    return rrt.get_node_list()

def check_intersection(line_start, line_end, circle_pos, r):
    """
    def line(sol):
        return [sol[0] * line_start[0] + sol[1] - line_start[1], sol[0] * line_end[0] + sol[1] - line_end[1]]
    sol = fsolve(line, [circle_pos[0]-r, circle_pos[1]-r])
    k = sol[0]
    b = sol[1]
    """

    def center_line(sol):
        return [sol[0] * line_start[0] + sol[1] - line_start[1], sol[0] * circle_pos[0] + sol[1] - circle_pos[1]]
    result = fsolve(center_line, [circle_pos[0] - r, circle_pos[1] - r])

    # cos(beta)
    OA = circle_pos - line_start
    BA = line_end - line_start
    l_oa = np.linalg.norm(OA)
    l_ba = np.linalg.norm(BA)
    cos_beta = np.dot(OA, BA) / (l_oa * l_ba)
    AD = l_oa * cos_beta
    OD = np.sqrt(np.power(l_oa, 2) - np.power(AD, 2))

    if OD <= r:
        result = True
    else:
        result = False

    return result

def draw_lines(line_seg, obstacle_pos, obstacle_r, start, goal, path):
    l_c = lc(line_seg)
    l_c_path = lc(path, color = 'r')
    f, ax = plt.subplots()
# draw a list of circles
    for i in range(len(obstacle_pos)):
        circle = plt.Circle(obstacle_pos[i], obstacle_r[i], fill=True)
        ax.add_artist(circle)

    ax.set_aspect(1)

    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.add_collection(l_c, '-o')
    ax.add_collection(l_c_path, '-o')
    
    plt.plot(start[0], start[1], 'ro')
    plt.plot(goal[0], goal[1], 'go')
    plt.show()

def random_circle(num_of_circles):
    position = []
    r = []
    for _ in range(num_of_circles):
        pos = np.random.randint(0, 100, (2,))
        position.append(pos)
        radi = np.random.randint(5, 10)
        r.append(radi)

    return position, r

def path_finder(start: Node, end: Node):
    path = [end]
    parent = end.get_parent()
    parent_pos = parent.get_pos()
    start_pos = start.get_pos()
    while parent_pos[0] != start_pos[0] and parent_pos[1] != start_pos[1]:
        path.append(parent)
        parent = parent.get_parent()
        parent_pos = parent.get_pos()

    return path

def test():
    
    circle_pos, circle_r = random_circle(12)

    delt = 1
    D = 100
    K = 800

    q_init = np.array([10, 10])
    goal = np.array([70, 70])
    node_list = obstacle_avoidance(circle_pos, circle_r, q_init, K, delt, D, goal)
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

    draw_lines(line_seg, obstacle_pos=circle_pos, obstacle_r=circle_r, start=q_init, goal=goal, path=path_seg)
        
if __name__ == "__main__":
    test()
