def cal(world_rob_x, world_rob_y, world_rob_theta, world_target_x, world_target_y):
    import numpy as np

    print("/////////////////////////////////////////////////")
    pr_x = world_target_x - world_rob_x
    pr_y = world_target_y - world_rob_y
    pr = np.array([pr_x, pr_y])

    if world_rob_theta < 0:
        cos = np.cos(world_rob_theta)
        sin = np.sin(world_rob_theta)
        rotate = np.array([[cos, -sin], [sin, cos]])

    if world_rob_theta >= 0:
        cos = np.cos(world_rob_theta)
        sin = np.sin(world_rob_theta)
        rotate = np.array([[cos, sin], [-sin, cos]])

    rob_target_position = np.dot(rotate, pr)
    rob_target_x = rob_target_position[0]
    rob_target_y = rob_target_position[1]

    print("world_rob_x  is " + str(world_rob_x))
    print("world_rob_y  is " + str(world_rob_y))

    print("world_target_x  is " + str(world_target_x))
    print("world_target_y  is " + str(world_target_y))


    radius = ((rob_target_x**2) + (rob_target_y**2) ) / (2 * rob_target_y)
    if radius > 1000000:
        radius = 100000

    print("radius is " + str(radius))
    print("/////////////////////////////////////////////////")
    return radius
