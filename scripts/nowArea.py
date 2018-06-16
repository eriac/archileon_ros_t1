def judge(world_rob_x, world_rob_y, now_target_num):
    import getWayPoints
    import getNearestPoint
    result = False
    way_points = getWayPoints.read_rob()

    if now_target_num == 0:
        print("Zero")
        world_rob_x > 1.001
        result = True
    elif now_target_num == 1:
        print("One")
        world_rob_y > 0.501
        result = True
    elif now_target_num == 2:
        print("Two")
        world_rob_x > 1.001
        result = True
    elif now_target_num == 3:
        print("Three")
        world_rob_x < 0.001
        result = True
    elif now_target_num == 4:
        print("Four")
        world_rob_y < 0.501
        result = True
    elif now_target_num == 5:
        print("Five")
        world_rob_x > 0.501
        result = True
    return result
