from rrt import RRT
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

def obstacle_avoidance(obstacle_pos: list, obstacle_r: list, q_init, k, delt, domain):
    rrt = RRT(q_init, k, delt, domain)

    node_num = rrt.get_node_num()
    while node_num <= k:
        
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
        rrt.expand(nearest_node, new_pos)
        node_num = rrt.get_node_num()

    return rrt.get_node_list()

def draw_lines(line_seg, obstacle_pos, obstacle_r):
    l_c = lc(line_seg)
    f, ax = plt.subplots()

    # draw a list of circles
    for i in range(len(obstacle_pos)):
        circle = plt.Circle(obstacle_pos[i], obstacle_r[i], fill=False)
        ax.add_artist(circle)

    ax.set_aspect(1)

    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.add_collection(l_c, '-o')
    
    plt.show()

def test():
    
    q_init = np.array([50, 50])
    delt = 1
    D = 100
    K = 300

    circle_pos = [np.array([60, 30]), np.array([40, 60]), np.array([30, 30])]
    circle_r = [10, 10, 10]

    node_list = obstacle_avoidance(circle_pos, circle_r, q_init, K, delt, D)
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

    draw_lines(line_seg, obstacle_pos=circle_pos, obstacle_r=circle_r)
        
if __name__ == "__main__":
    test()