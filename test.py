from tabnanny import check
from rrt import RRT
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection as lc
from scipy.optimize import fsolve
import imageio.v3 as iio
from task_3 import check_collision

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

    start = np.array([14.8, 10.3])
    end = np.array([80, 37])
    start = np.asarray(start, dtype=int)
    c = check_collision(start, end, obs_loc)
    print(c)

    # a = np.array([2, 3])
    # b = np.array([[2, 3], [2,2], [1,2]])
    # c = np.linalg.norm(b - a, axis=1)
    # print(c)

    """
    a = np.zeros((10,10))
    a[2:8, 2:8] = 1
    print(a)

    obs = np.where(a == 1)
    obs = np.vstack((obs[0], obs[1])).T
    obs_loc = np.zeros(obs.shape)
    obs_loc[:, 0] = obs[:, 1]
    obs_loc[:, 1] = obs[:, 0]

    pt_start = np.array([8, 4])
    pt_end = np.array([1, 6])
    check = check_collision(pt_start, pt_end, obs_loc)
    a[2, 1] = 8
    a[2, 9] = 8

    print(check)
    print(a)

    a = np.array([
        [1,1],
        [2,2],
        [5,5]
    ])

    #plt.plot(a, 'o')
    img = iio.imread('N_map.png')
    plt.imshow(img)
    plt.show()


    
    f, ax = plt.subplots()
    circle = plt.Circle([2, 2], 1, fill = False)

    ax.set_xlim(0, 5)
    ax.set_ylim(0, 5)
    ax.set_aspect(1)
    # plt.show()

    # p, k, b = detect_collision(np.array([1.5,2.5]), np.array([2,2]), 1)
    
    # x = np.arange(0, 4, 0.05)
    # plt.plot(x, k*x+b, 'o')
    # plt.plot(p[0], p[1], 'or')
    ax.add_artist(circle)

    a = np.array([1, 1])
    b = np.array([4, 4])
    check = check_intersection(a, b, np.array([2, 2]), 1)
    if check:
        print("intersected")
    else:
        print("not intersected")

    #x = np.arange(0, 4, 0.05)
    #plt.plot(x, k*x+c, 'o')
    # plt.plot(p[0], p[1], 'or')
    plt.plot([a[0], b[0]], [a[1], b[1]], 'go-')
    #plt.plot(root, 'o')
    plt.show()
    

    #print(r)
    """

    """
    x = np.ones((2, 2))
    y = 2 * np.ones((2, 2))
    mat = np.stack((x, y), axis=2)
    x = y - x
    z = x[np.where(x == 0)]
    check = z.size == 0
    print(check)
    #print(mat)
    #print("mat shape")
    #print(mat.shape)
    """





