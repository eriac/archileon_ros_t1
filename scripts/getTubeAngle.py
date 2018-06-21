def cal(origin_x, origin_y, p1_x, p1_y, p2_x, p2_y):
    import numpy as np
    import math

    op1_x =p1_x-origin_x
    op1_y =p1_y-origin_y

    op2_x =p2_x-origin_x
    op2_y =p2_y-origin_y

    op1_vector = np.array([op1_x, op1_y], dtype= np.float32)
    norm_op1_vector = np.linalg.norm(op1_vector)

    op2_vector = np.array([op2_x, op2_y], dtype= np.float32)
    norm_op2_vector = np.linalg.norm(op2_vector)

    cos_theta = np.dot(op1_vector, op2_vector) / (norm_op1_vector * norm_op2_vector)
    radian = math.acos(cos_theta)
    return radian
