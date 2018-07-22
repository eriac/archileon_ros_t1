def cal(u_x, u_y, v_x, v_y):
    import numpy as np
    import math
    # http: // www5d.biglobe.ne.jp/~noocyte/Programming/Geometry/RotationDirection.html
    s = u_x * v_y - u_y * v_x

    vector_u = np.array([u_x, u_y])
    vector_v = np.array([v_x, v_y])

    norm_vector_u = np.linalg.norm(vector_u)
    norm_vector_v = np.linalg.norm(vector_v)

    if (norm_vector_u * norm_vector_v) != 0:
        cos_theta = np.dot(vector_u, vector_v) / \
            (norm_vector_u * norm_vector_v)
        if s > 0:
            radian = math.acos(cos_theta)
        else:
            radian = -math.acos(cos_theta)
    else:
        radian = 0

    return radian
