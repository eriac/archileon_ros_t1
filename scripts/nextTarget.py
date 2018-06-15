def judge(world_rob_x, world_rob_y, now_target_num):
    import read_way_points
    import getNearestPoint
    result = False
    # way_points = read_way_points.read_points()
    # base_point_num, adjacent_point_num = getNearestPoint.search_value(way_points, world_rob_x, world_rob_y)
    #
    # base_point_x = way_points[base_point_num][0]
    # base_point_y = way_points[base_point_num][1]

    if now_target_num == 0:
        print("Zero")
        world_rob_x > 1.005
        result = True
    elif now_target_num == 1:
        print("One")
        world_rob_y > 0.505
        result = True
    elif now_target_num == 2:
        print("Two")
        world_rob_x > 1.005
        result = True
    elif now_target_num == 3:
        print("Three")
        world_rob_x < 0.005
        result = True
    elif now_target_num == 4:
        print("Four")
        world_rob_y < 0.505
        result = True
    elif now_target_num == 5:
        print("Five")
        world_rob_x > 0.505
        result = True
    return result
