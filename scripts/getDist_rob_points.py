def cal(point_1_x, point_1_y, point_2_x, point_2_y, world_rob_x, world_rob_y):
    import numpy as np
    x1 = point_1_x
    y1 = point_1_y
    x2 = point_2_x
    y2 = point_2_y
    x3 = world_rob_x
    y3 = world_rob_y

    u = np.array([x2 - x1, y2 - y1])
    v = np.array([x3 - x1, y3 - y1])
    L = np.cross(u, v) / np.linalg.norm(u)
    return L
