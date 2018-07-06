def cal(world_rob_x, world_rob_y, world_rob_theta, world_target_x, world_target_y):
    import numpy as np
    import math
    
    w_rob_x_target_x = world_target_x - world_rob_x
    w_rob_y_target_y = world_target_y - world_rob_y
    vector_rob_target = np.array([w_rob_x_target_x, w_rob_y_target_y])

    cos = np.cos(world_rob_theta)
    sin = np.sin(world_rob_theta)
    rotate = np.array([[cos, sin], [-sin, cos]])

    rob_target_position = np.dot(rotate, vector_rob_target)
    rob_target_x = rob_target_position[0]
    rob_target_y = rob_target_position[1]

    
    radius = ((rob_target_x**2) + (rob_target_y**2)) / (2 * rob_target_y)

    if abs(radius) > 100:
        dist_rob_target = math.sqrt(rob_target_x**2 + rob_target_y**2)
        print("Adist_rob_target " +str(dist_rob_target))
    else:
        center_x = 0.0
        center_y = radius

        rob_rob_x = 0.0
        rob_rob_y = 0.0
            
        center_x_rob_x = rob_rob_x - center_x 
        center_y_rob_y = rob_rob_y - center_y 
        vector_u = np.array([center_x_rob_x, center_y_rob_y])
        norm_vector_u = float(np.linalg.norm(vector_u))

        center_x_target_x = rob_target_x - center_x
        center_y_target_y = rob_target_y - center_y
        vector_v = np.array([center_x_target_x, center_y_target_y])
        norm_vector_v = float(np.linalg.norm(vector_v))


        u_dot_v = float(np.dot(vector_u, vector_v))
        # cos = float((norm_vector_u * norm_vector_v) / u_dot_v)


        theta = np.arccos(float(np.linalg.norm(vector_u) * np.linalg.norm(vector_v)) /
        float(np.dot(vector_u, vector_v)))

        dist_rob_target = radius * theta
        print("Bdist_rob_target " +str(dist_rob_target))
        print("u_dot_v " +str(u_dot_v))
        print("norm_vector_u " +str(norm_vector_u))
        print("norm_vector_v " +str(norm_vector_v))
                

        print("theta " +str(theta))


    print("rob_target_x " +(str(rob_target_x)))
    print("rob_target_y " +(str(rob_target_y)))

    return dist_rob_target
