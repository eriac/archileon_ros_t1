def cal(tube_rot_axis_position_x, tube_rot_axis_position_y, tube_tip_position_x, tube_tip_position_y, world_inter_section_x, world_inter_section_y):
    import numpy as np
    import math
    tube_rot_axis_position_x = tube_rot_axis_position_x
    tube_rot_axis_position_y = tube_rot_axis_position_y

    tube_tip_position_x = tube_tip_position_x
    tube_tip_position_y = tube_tip_position_y

    inter_section_point_x = world_inter_section_x
    inter_section_point_y = world_inter_section_y

    rob_rotaxis_to_tip_x = tube_tip_position_x - tube_rot_axis_position_x
    rob_rotaxis_to_tip_y = tube_tip_position_y - tube_rot_axis_position_y

    rob_rotaxis_tip = np.array([rob_rotaxis_to_tip_x, rob_rotaxis_to_tip_y])
    norm_rob_rotaxis_tip = np.linalg.norm(rob_rotaxis_tip)

    rob_rotaxis_to_tip_x = inter_section_point_x - tube_rot_axis_position_x
    rob_rotaxis_to_tip_y = inter_section_point_y - tube_rot_axis_position_y

    rob_rotaxis_inter = np.array([rob_rotaxis_to_tip_x, rob_rotaxis_to_tip_y])
    norm_rob_rotaxis_inter = np.linalg.norm(rob_rotaxis_inter)

    cos_theta = np.dot(rob_rotaxis_tip, rob_rotaxis_inter) / (norm_rob_rotaxis_tip * norm_rob_rotaxis_inter)
    radian = math.acos(cos_theta)
    return radian
