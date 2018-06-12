def judge(temporal_world_rob_x, temporal_world_rob_y, world_goal_x, world_goal_y):
    result = False
    diff_x = abs(world_goal_x - temporal_world_rob_x)
    diff_y = abs(world_goal_y - temporal_world_rob_y)
    return result
