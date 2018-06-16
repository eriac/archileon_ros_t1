def judge(world_rob_x, world_rob_y, now_target_num):
    import getWayPoints
    import getNearestPoint
    result = False
    way_points = getWayPoints.read_rob()

    # print("now_target_num " +str(now_target_num))

    if now_target_num == 0:
        print("Zero")
        if world_rob_x > 1.001:
            result = True
    # elif now_target_num == 1:
    #     print("One")
    #     if world_rob_y > 0.501:
    #         result = True
    # elif now_target_num == 2:
    #     print("Two")
    #     if world_rob_x > 1.001:
    #         result = True
    # elif now_target_num == 3:
    #     print("Three")
    #     if world_rob_x < 0.001:
    #         result = True
    # elif now_target_num == 4:
    #     print("Four")
    #     if world_rob_y < 0.501:
    #         result = True
    # elif now_target_num == 5:
    #     print("Five")
    #     if world_rob_x > 0.501:
    #         result = True
    return result
