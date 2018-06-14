def judge(world_rob_x, world_rob_y, world_rob_theta, target_point):
    import read_way_points
    import cal_move_curve

    result = False
    min_curve = 0.3
    target_point_x = target_point[0]
    target_point_y = target_point[1]

    move_curve = cal_move_curve.cal(world_rob_x, world_rob_y, world_rob_theta, world_target_x=target_point_x, world_target_y=target_point_y)

    if move_curve < min_curve:
        result = True
    return result
