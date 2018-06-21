def cal(world_rob_x, world_rob_y, world_rob_theta, world_target_x, world_target_y):
    import numpy as np

    rx_px = world_target_x - world_rob_x
    ry_py = world_target_y - world_rob_y
    r_p = np.array([rx_px, ry_py])

    if world_rob_theta < 0:
        cos = np.cos(world_rob_theta)
        sin = np.sin(world_rob_theta)
        rotate = np.array([[cos, -sin], [sin, cos]])

    elif world_rob_theta >= 0:
        cos = np.cos(world_rob_theta)
        sin = np.sin(world_rob_theta)
        rotate = np.array([[cos, sin], [-sin, cos]])

    rob_target_position = np.dot(rotate, r_p)
    rob_target_x = rob_target_position[0]
    rob_target_y = rob_target_position[1]


    radius = ((rob_target_x**2) + (rob_target_y**2) ) / (2 * rob_target_y)
    if radius > 1000000:
        radius = 100000
    return radius
