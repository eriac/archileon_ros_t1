# coding: UTF-8
def cal(w_rob_x, w_rob_y, w_rob_theta):
    import numpy as np

    w_rob_pos = np.array([w_rob_x, w_rob_y])

    cos = np.cos(w_rob_theta)
    sin = np.sin(w_rob_theta)

    rotate = np.array([[cos, sin], [-sin, cos]])

    r_rob_pos = np.dot(rotate, w_rob_pos)

    r_bl_tube_rot_pos_x = r_rob_pos[0] - 0.1258
    r_bl_tube_rot_pos_y = r_rob_pos[1] + 0.075
    r_bl_tube_pos_x = r_rob_pos[0] - 0.23
    r_bl_tube_pos_y = r_rob_pos[1] + 0.075

    r_br_tube_rot_pos_x = r_rob_pos[0] - 0.1258
    r_br_tube_rot_pos_y = r_rob_pos[1] - 0.075
    r_br_tube_pos_x = r_rob_pos[0] - 0.23
    r_br_tube_pos_y = r_rob_pos[1] - 0.075

    r_bl_tube_rot_pos = np.array([r_bl_tube_rot_pos_x, r_bl_tube_rot_pos_y])
    r_bl_tube_pos = np.array([r_bl_tube_pos_x, r_bl_tube_pos_y])

    r_br_tube_rot_pos = np.array([r_br_tube_rot_pos_x, r_br_tube_rot_pos_y])
    r_br_tube_pos = np.array([r_br_tube_pos_x, r_br_tube_pos_y])

    reverse_rotate = np.array([[cos, -sin], [sin, cos]])

    w_bl_rot_pos = np.dot(reverse_rotate, r_bl_tube_rot_pos)
    w_bl_tube_pos = np.dot(reverse_rotate, r_bl_tube_pos)

    w_br_rot_pos = np.dot(reverse_rotate, r_br_tube_rot_pos)
    w_br_tube = np.dot(reverse_rotate, r_br_tube_pos)

    return w_bl_rot_pos, w_bl_tube_pos, w_br_rot_pos, w_br_tube
