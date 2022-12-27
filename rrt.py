import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection as lc
import random 

class Node:
    def __init__(self, pos):
        self.parent = None
        self.child = []
        self.pos = pos

    def get_pos(self):
        return self.pos

    def get_child(self):
        return self.child

    def get_parent(self):
        return self.parent

    def set_child(self, child):
        self.child.append(child)

    def set_parent(self, parent):
        self.parent = parent

    def set_pos(self, pos):
        self.pos = pos

class RRT:
    def __init__(self, q_init, k, delt, domain):
        self.q_init = q_init
        self.k = k
        self.delt = delt
        self.domain = domain

        # dictionary of nodes:
        self.node_pos_dict = {}

        root = Node(q_init)
        self.node_list = [root]

    # return the latest node in the tree
    def get_latest_node(self):
        return self.node_list[len(self.node_list) - 1]

    def get_new_pos(self):
        # random reference pos
        x_pos = random.uniform(0, 100)
        y_pos = random.uniform(0, 100)

        ref_pos = np.array([x_pos, y_pos])
        nearest_node = self.find_nearest_node(ref_pos)

        # angle between ref and horizontal:
        ref_curr_vec = ref_pos - nearest_node.get_pos()
        ref_curr_dist = np.linalg.norm(ref_curr_vec)

        # pos of new node:
        new_pos = nearest_node.get_pos() + np.array([ref_curr_vec[0]/ref_curr_dist, ref_curr_vec[1]/ref_curr_dist])
        return nearest_node, new_pos

    # add random num of random nodes to current node
    def expand(self, nearest_node, new_pos):
        # new node:
        new_node = Node(new_pos)
        nearest_node.set_child(new_node)
        new_node.set_parent(nearest_node)
        self.node_list.append(new_node)

        return new_node
        
    def find_nearest_node(self, ref_pt):
        dist_dict = {}
        dist_list = []
        # list of dist:
        for node in self.node_list:
            dist = np.linalg.norm(ref_pt - node.get_pos())
            dist_dict[dist] = node
            dist_list.append(dist)

        dist_list.sort()
        nearest_node = dist_dict[dist_list[0]]

        return nearest_node
        


    def get_node_num(self):
        return len(self.node_list)

    def get_init_pos(self):
        return self.q_init

    def get_node_dict(self):
        return self.node_pos_dict

    def get_node_list(self):
        return self.node_list


def expand_test():

    q_init = np.array([50, 50])
    delt = 1
    D = 100
    K = 300
    
    simple_rrt = RRT(q_init, K, delt, D)

    # expand the tree:
    node_num = simple_rrt.get_node_num()
    curr_pos = q_init
    curr_node = Node(curr_pos)
    # new_nodes_list = [curr_node]
    while node_num < K:
        simple_rrt.expand()
        node_num = simple_rrt.get_node_num()

    print("finish tree expansion")

    return simple_rrt.get_node_dict(), simple_rrt.get_node_list()
    
def draw_lines(line_seg):
    l_c = lc(line_seg)
    f, ax = plt.subplots()

    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.add_collection(l_c, '-o')
    
    plt.show()

if __name__ == "__main__":
    
    nodes_pos_dict, nodes_list = expand_test()
    nodes_pos_x = []
    nodes_pos_y = []
    line_seg = []



    for node in nodes_list:
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

    #plt.plot(nodes_pos_x, nodes_pos_y, 'o')
    #plt.show()

    draw_lines(line_seg)