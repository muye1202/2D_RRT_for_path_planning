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
    return closest_point, k, b

    if dot_product <= 0:
        return True
    else:
        return False

    



if __name__ == "__main__":
    f, ax = plt.subplots()
    circle = plt.Circle([2, 2], 1, fill = False)

    ax.set_xlim(0, 5)
    ax.set_ylim(0, 5)
    ax.set_aspect(1)
    # plt.show()

    p, k, b = detect_collision(np.array([1.5,2.5]), np.array([2,2]), 1)
    
    x = np.arange(0, 4, 0.05)
    plt.plot(x, k*x+b, 'o')
    plt.plot(p[0], p[1], 'or')
    ax.add_artist(circle)
    plt.show()
    

    #print(r)





