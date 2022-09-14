from unittest import result
from rrt import RRT, Node
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection as lc
from scipy.optimize import fsolve

def detect_collision(pt_pos, circle_pos, r):
    # generate points on the circle
    # eqn for circle: (x + cx)^2 + (y + cy)^2 = r^2
    # solve for intersection
    # first get the line connecting center of circle to vertex
    # sol[0] = k, sol[1] = b
    def line(sol):
        return [sol[0] * pt_pos[0] + sol[1] - pt_pos[1], sol[0] * circle_pos[0] + sol[1] - circle_pos[1]]
    sol = fsolve(line, [circle_pos[0]-r, circle_pos[1]-r])
    k = sol[0]
    b = sol[1]

    # solve for intersection between the line and the circle
    # x[0] = x, x[1] = y
    def circle_eqn(s):
        return [ np.power((s[0] - circle_pos[0]), 2) + np.power((s[1] - circle_pos[1]), 2) - np.power(r, 2), k*s[0] + b - s[1]]
    if pt_pos[0] <= circle_pos[0]:
        start_x = circle_pos[0] - r
        if pt_pos[1] >= circle_pos[1]:
            start_y = circle_pos[1]
        else:
            start_y = circle_pos[1] - r
    else:
        start_x = circle_pos[0] + r
        if pt_pos[1] >= circle_pos[1]:
            start_y = circle_pos[1]
        else:
            start_y = circle_pos[1] - r

    closest_point = fsolve(circle_eqn, [start_x, start_y])

    # use vector dot product to check
    circle_dir = closest_point - pt_pos
    center_dir = circle_pos - pt_pos
    dot_product = np.dot(circle_dir, center_dir)

    #return k, b
    #return closest_point, k, b

    if dot_product <= 0:
        return True
    else:
        return False

def obstacle_avoidance(obstacle_pos: list, obstacle_r: list, q_init, k, delt, domain, goal):
    rrt = RRT(q_init, k, delt, domain)
    node_num = rrt.get_node_num()

    #exist = False
    while node_num < k:
        
        flag = False
        # if collision happens, expand to other vertices
        while flag == False:
            # check whether new vertex is in obstacle
            collision = 0
            # new vertex:
            nearest_node, new_pos = rrt.get_new_pos()
            for i in range(len(obstacle_pos)):
                check = detect_collision(new_pos, obstacle_pos[i], obstacle_r[i])
                if check == True:
                    collision += 1

            if collision == 0:
                flag = True

        # if passed the above while loop
        new_node = rrt.expand(nearest_node, new_pos)
        new_pos = new_node.get_pos()
        # return if no obstacle between this node and goal
        collision = 0
        for i in range(len(obstacle_pos)):
            root = check_intersection(new_pos, goal, obstacle_pos[i], obstacle_r[i])
            if root[0] >= obstacle_pos[i][0] - obstacle_r[i] and root[0] <= obstacle_pos[i][0] + obstacle_r[i]:
                collision += 1

        if collision == 0:
            rrt.expand(new_node, goal)
            break
        
        node_num = rrt.get_node_num()

    return rrt.get_node_list()

def check_intersection(line_start, line_end, circle_pos, r):
    def line(sol):
        return [sol[0] * line_start[0] + sol[1] - line_end[1], sol[0] * circle_pos[0] + sol[1] - circle_pos[1]]
    sol = fsolve(line, [circle_pos[0]-r, circle_pos[1]-r])
    k = sol[0]
    b = sol[1]

    # solve for intersection between the line and the circle
    # x[0] = x, x[1] = y
    def circle_eqn(s):
        return [ np.power((s[0] - circle_pos[0]), 2) + np.power((s[1] - circle_pos[1]), 2) - np.power(r, 2), k*s[0] + b - s[1]]

    result = fsolve(circle_eqn, [circle_pos[0] - r, circle_pos[1] - r])

    return result

def draw_lines(line_seg, obstacle_pos, obstacle_r, start, goal, path):
    l_c = lc(line_seg)
    l_c_path = lc(path, color = 'r')
    f, ax = plt.subplots()

    # draw a list of circles
    for i in range(len(obstacle_pos)):
        circle = plt.Circle(obstacle_pos[i], obstacle_r[i], fill=False)
        ax.add_artist(circle)

    ax.set_aspect(1)

    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.add_collection(l_c, '-o')
    ax.add_collection(l_c_path, '-o')
    
    plt.plot(30, 30, 'ro')
    plt.plot(40, 90, 'go')
    plt.show()

def random_circle(num_of_circles):
    position = []
    r = []
    for _ in range(num_of_circles):
        pos = np.random.randint(40, 80, (2,))
        position.append(pos)
        radi = np.random.randint(10, 21)
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
    
    q_init = np.array([30, 30])
    delt = 1
    D = 100
    K = 600
    goal = np.array([40, 90])

    circle_pos, circle_r = random_circle(5)

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