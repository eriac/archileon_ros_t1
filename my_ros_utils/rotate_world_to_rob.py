def cal(world_origin_x, world_origin_y, world_origin_theta, world_target_x, world_target_y):
    import numpy as np

    w_rob_x_target_x = world_target_x - world_origin_x
    w_rob_y_target_y = world_target_y - world_origin_y

    vector_origin_to_target = np.array([w_rob_x_target_x, w_rob_y_target_y])

    cos = float(np.cos(world_origin_theta))
    sin = float(np.sin(world_origin_theta))
    rotate = np.array([[cos, sin], [-sin, cos]])

    con_vector_origin_to_target = np.dot(rotate, vector_origin_to_target)
    return con_vector_origin_to_target


def reversecal(r_rob_x, r_rob_y, w_rob_theta, r_target_x, r_target_y):
    import numpy as np

    r_rob_x_target_x = r_target_x - r_rob_x
    r_rob_y_target_y = r_target_y - r_rob_y
    vector_origin_to_target = np.array([r_rob_x_target_x, r_rob_y_target_y])

    cos = float(np.cos(w_rob_theta))
    sin = float(np.sin(w_rob_theta))
    reverse_rotate = np.array([[cos, -sin], [sin, cos]])

    world_target_position = np.dot(reverse_rotate, vector_origin_to_target)
    return world_target_position
