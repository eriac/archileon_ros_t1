def cal(world_rob_x, world_rob_y, world_rob_theta):
    import numpy as np

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

    # 回転の軸はBLのタイヤから125.8mm後ろ
    rob_bl_tube_rot_axis_position_x = rob_rob_x - 0.1258
    rob_bl_tube_rot_axis_position_y = rob_rob_y + 0.075
    rob_bl_tube_tip_position_x = rob_rob_x - 0.23
    rob_bl_tube_tip_position_y = rob_rob_y + 0.075

    # 回転の軸はBLのタイヤから125.8mm後ろ
    rob_br_tube_rot_axis_position_x = rob_rob_x - 0.1258
    rob_br_tube_rot_axis_position_y = rob_rob_y - 0.075
    rob_br_tube_tip_position_x = rob_rob_x - 0.23
    rob_br_tube_tip_position_y = rob_rob_y - 0.075

    rob_bl_tube_rot_axis = np.array([rob_bl_tube_rot_axis_position_x, rob_bl_tube_rot_axis_position_y])
    rob_bl_tube_tip = np.array([rob_bl_tube_tip_position_x, rob_bl_tube_tip_position_y])
    rob_br_tube_rot_axis = np.array([rob_br_tube_rot_axis_position_x, rob_br_tube_rot_axis_position_y])
    rob_br_tube_tip = np.array([rob_br_tube_tip_position_x, rob_br_tube_tip_position_y])


    if world_rob_theta >= 0:
        reverse_rotate = np.array([[cos, -sin], [sin, cos]])
    elif world_rob_theta < 0:
        reverse_rotate = np.array([[cos, sin], [-sin, cos]])

    world_rob_bl_tube_rot_axis_position = np.dot(reverse_rotate, rob_bl_tube_rot_axis)
    world_rob_bl_tube_tip_position = np.dot(reverse_rotate, rob_bl_tube_tip)
    world_br_tube_rot_axis_position = np.dot(reverse_rotate, rob_br_tube_rot_axis)
    world_br_tube_tip_position = np.dot(reverse_rotate, rob_br_tube_tip)

    return world_rob_bl_tube_rot_axis_position, world_rob_bl_tube_tip_position,world_br_tube_rot_axis_position,world_br_tube_tip_position
