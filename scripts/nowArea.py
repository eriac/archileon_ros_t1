def judge(world_rob_x, world_rob_y, now_target_num, way_points):
    import getWayPoints
    import getNearestPoint
    result = False
    way_points = way_points

    # print("now_target_num " +str(now_target_num))

    if now_target_num == 0:
        if world_rob_x > 1.0001:
            result = True
    elif now_target_num == 1:
        if world_rob_y > 0.5001:
            result = True
    elif now_target_num == 2:
        if world_rob_x > 1.0001:
            result = True
    elif now_target_num == 3:
        if world_rob_x < 0.0001:
            result = True
    elif now_target_num == 4:
        if world_rob_y < 0.5001:
            result = True
    elif now_target_num == 5:
        if world_rob_x > 0.5001:
            result = True
    return result
