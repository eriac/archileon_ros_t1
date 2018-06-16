def cal(world_rob_x, world_rob_y, world_rob_theta):
    import numpy as np

    print("/////////////////////////////////////////////////")
    print("world_rob_x  is " + str(world_rob_x))
    print("world_rob_y  is " + str(world_rob_y))

    # 原点からロボまでの距離のベクトル
    ox_rx = world_rob_x - 0.0
    oy_ry = world_rob_y - 0.0

    world_rob_position = np.array([ox_rx, oy_ry])

    cos = np.cos(world_rob_theta)
    sin = np.sin(world_rob_theta)

    if world_rob_theta >= 0:
        rotate = np.array([[cos, sin], [-sin, cos]])
    if world_rob_theta < 0:
        rotate = np.array([[cos, -sin], [sin, cos]])

    rob_rob_position = np.dot(rotate, world_rob_position)
    rob_rob_x = rob_rob_position[0]
    rob_rob_y = rob_rob_position[1]

    print("rob_rob_x  is " + str(rob_rob_x))
    print("rob_rob_y  is " + str(rob_rob_y))

    # 本当は0.1258
    rob_bl_tube_rot_axis_position_x = rob_rob_x - 0.125
    rob_bl_tube_rot_axis_position_y = rob_rob_y + 0.075
    rob_bl_tube_tip_position_x = rob_rob_x - 0.23
    rob_bl_tube_tip_position_y = rob_rob_y + 0.075

    # 本当は0.1258
    rob_br_tube_rot_axis_position_x = rob_rob_x - 0.125
    rob_br_tube_rot_axis_position_y = rob_rob_y - 0.075
    rob_br_tube_tip_position_x = rob_rob_x - 0.23
    rob_br_tube_tip_position_y = rob_rob_y -0.075

    world_bl_tube_rot_axis = np.array([rob_bl_tube_rot_axis_position_x, rob_bl_tube_rot_axis_position_y])
    world_bl_tube_tip = np.array([rob_bl_tube_tip_position_x, rob_bl_tube_tip_position_y])

    world_br_tube_rot_axis = np.array([rob_br_tube_rot_axis_position_x, rob_br_tube_rot_axis_position_y])
    world_br_tube_tip = np.array([rob_br_tube_tip_position_x, rob_br_tube_tip_position_y])


    if world_rob_theta >= 0:
        reverse_rotate = np.array([[cos, -sin], [sin, cos]])
    elif world_rob_theta < 0:
        reverse_rotate = np.array([[cos, sin], [-sin, cos]])

    rob_bl_tube_rot_axis_position = np.dot(reverse_rotate, world_bl_tube_rot_axis)
    rob_bl_tube_tip_position = np.dot(reverse_rotate, world_bl_tube_tip)

    print(rob_bl_tube_rot_axis_position)
    print(rob_bl_tube_tip_position)

    rob_br_tube_rot_axis_position = np.dot(reverse_rotate, world_br_tube_rot_axis)
    rob_br_tube_tip_position = np.dot(reverse_rotate, world_br_tube_tip)

    print(rob_br_tube_rot_axis_position)
    print(rob_br_tube_tip_position)



    print("/////////////////////////////////////////////////")
    return rob_bl_tube_rot_axis_position, rob_bl_tube_tip_position,rob_br_tube_rot_axis_position,rob_br_tube_tip_position
