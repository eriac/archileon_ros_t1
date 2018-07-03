def cal(world_rob_x, world_rob_y, world_rob_theta, world_target_x, world_target_y):
    import numpy as np

    w_rob_x_target_x = world_target_x - world_rob_x
    w_rob_y_target_y = world_target_y - world_rob_y
    vector_rob_target = np.array([w_rob_x_target_x, w_rob_y_target_y])

    if world_rob_theta < 0:
        cos = np.cos(world_rob_theta)
        sin = np.sin(world_rob_theta)
        rotate = np.array([[cos, -sin], [sin, cos]])

    elif world_rob_theta >= 0:
        cos = np.cos(world_rob_theta)
        sin = np.sin(world_rob_theta)
        rotate = np.array([[cos, sin], [-sin, cos]])

    rob_target_position = np.dot(rotate, vector_rob_target)
    rob_target_x = rob_target_position[0]
    rob_target_y = rob_target_position[1]
    
    radius = ((rob_target_x**2) + (rob_target_y**2)) / (2 * rob_target_y)
    if radius > 100:
        radius = 100

    return radius
