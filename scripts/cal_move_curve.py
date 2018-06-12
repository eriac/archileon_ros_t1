def cal(world_rob_x, world_rob_y, world_rob_theta, world_target_x, world_target_y):
    import numpy as np

    pr_x = world_target_x - world_rob_x
    pr_y = world_target_y - world_rob_y
    pr = np.array([pr_x, pr_y])

    if world_rob_theta < 0:
        print("world_rob_theta is MINUS" + str(world_rob_theta))
        print(" ")
        cos = np.cos(world_rob_theta)
        sin = np.sin(world_rob_theta)
        rotate = np.array([[cos, -sin], [sin, cos]])

    if world_rob_theta >= 0:
        print("world_rob_theta is PLUS" + str(world_rob_theta))
        print(" ")
        cos = np.cos(world_rob_theta)
        sin = np.sin(world_rob_theta)
        rotate = np.array([[cos, sin], [-sin, cos]])

    rob_target_position = np.dot(rotate, pr)
    rob_target_x = rob_target_position[0]
    rob_target_y = rob_target_position[1]

    print("rob_target_x  is " + str(rob_target_x))
    print("rob_target_y  is " + str(rob_target_y))
    print(" ")


    radius = ((rob_target_x**2) + (rob_target_y**2) ) / (2 * rob_target_y)
    if radius > 1000000:
        radius = 100000
    print("radius is " + str(radius))


    move_curve = radius
    print("曲率" + str(1.0 / move_curve))
    print("/////////////////////////////////////////////////")
    return move_curve
