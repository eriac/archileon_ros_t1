def cal(origin_x, origin_y, u_x, u_y, v_x, v_y):
    import numpy as np
    import math

    el_u_x =u_x - origin_x
    el_u_y =u_y - origin_y
    el_v_x =v_x - origin_x
    el_v_y =v_y - origin_y

    vector_u = np.array([el_u_x, el_u_y])
    norm_vector_u = np.linalg.norm(vector_u)
    vector_v = np.array([el_v_x, el_v_y])
    norm_vector_v = np.linalg.norm(vector_v)


    if (norm_vector_u * norm_vector_v) != 0:
        cos_theta = np.dot(vector_u, vector_v) / (norm_vector_u * norm_vector_v)
        radian = math.acos(cos_theta)
    else:
        radian = 0

    return radian


def change_w_r_axis(world_rob_x, world_rob_y, world_rob_theta, world_target_x, world_target_y):
    import numpy as np

    w_rob_x_target_x = world_target_x - world_rob_x
    w_rob_y_target_y = world_target_y - world_rob_y
    vector_rob_target = np.array([w_rob_x_target_x, w_rob_y_target_y])

    cos = np.cos(world_rob_theta)
    sin = np.sin(world_rob_theta)
    rotate = np.array([[cos, sin], [-sin, cos]])

    rob_target_position = np.dot(rotate, vector_rob_target)
    rob_target_x = rob_target_position[0]
    rob_target_y = rob_target_position[1]
    
    radius = ((rob_target_x**2) + (rob_target_y**2)) / (2 * rob_target_y)

    if radius > 100:
        radius = 100
    elif radius < -100:
        radius = -100


    return radius
