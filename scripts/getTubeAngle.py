def cal(origin_x, origin_y, u_x, u_y, v_x, v_y):
    import numpy as np
    import math

    el_u_x =u_x - origin_x
    el_u_y =u_y - origin_y
    el_v_x =v_x - origin_x
    el_v_y =v_y - origin_y

    vector_u = np.array([el_u_x, el_u_y], dtype= np.float32)
    norm_vector_u = np.linalg.norm(vector_u)
    vector_v = np.array([el_v_x, el_v_y], dtype= np.float32)
    norm_vector_v = np.linalg.norm(vector_v)


    cos_theta = np.dot(vector_u, vector_v) / (norm_vector_u * norm_vector_v)
    radian = math.acos(cos_theta)
    return radian
