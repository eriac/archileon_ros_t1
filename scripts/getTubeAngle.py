def cal(rob_tube_rot_axis_position, rob_tube_tip_position, inter_section_point):
    import numpy as np
    import math
    rob_tube_rot_axis_position_x = rob_tube_rot_axis_position[0]
    rob_tube_rot_axis_position_y = rob_tube_rot_axis_position[1]

    rob_tube_tip_position_x = rob_tube_tip_position[0]
    rob_tube_tip_position_y = rob_tube_tip_position[1]

    inter_section_point_x = inter_section_point[0]
    inter_section_point_y = inter_section_point[1]

    rob_rotaxis_to_tip_x = rob_tube_tip_position_x - rob_tube_rot_axis_position_x
    rob_rotaxis_to_tip_y = rob_tube_tip_position_y - rob_tube_rot_axis_position_y

    rob_rotaxis_tip = np.array([rob_rotaxis_to_tip_x, rob_rotaxis_to_tip_y])
    norm_rob_rotaxis_tip = np.linalg.norm(rob_rotaxis_tip)

    rob_rotaxis_to_tip_x = inter_section_point_x - rob_tube_rot_axis_position_x
    rob_rotaxis_to_tip_y = inter_section_point_y - rob_tube_rot_axis_position_y

    rob_rotaxis_inter = np.array([rob_rotaxis_to_tip_x, rob_rotaxis_to_tip_y])
    norm_rob_rotaxis_inter = np.linalg.norm(rob_rotaxis_inter)

    cos_theta = np.dot(rob_rotaxis_tip, rob_rotaxis_inter) / (norm_rob_rotaxis_tip * norm_rob_rotaxis_inter)
    radian = math.acos(cos_theta)
    return radian
