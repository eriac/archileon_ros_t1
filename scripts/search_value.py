def getNearestPoint(map_points, world_rob_x, world_rob_y):
    import math

    tmp_points_x = map_points[1][0]
    tmp_points_y = map_points[1][1]
    diff_x = world_rob_x - tmp_points_x
    diff_y = world_rob_y - tmp_points_y
    base_diff_xy = math.sqrt(diff_x**2 + diff_y**2)
    base_num = 0

    for i in range(len(map_points)):
        tmp_points_x = map_points[i][0]
        tmp_points_y = map_points[i][1]
        diff_x = world_rob_x - tmp_points_x
        diff_y = world_rob_y - tmp_points_y
        tmp_diff_xy = math.sqrt(diff_x**2 + diff_y**2)

        if tmp_diff_xy < base_diff_xy:
            base_num = i
            base_diff_xy = math.sqrt(diff_x**2 + diff_y**2)

        if base_num == 0:
            next_num = base_num + 1

        elif base_num == 1712:
            next_num = base_num - 1

        else:
            front_diff_xy = math.sqrt((world_rob_x - map_points[base_num + 1][0])**2+world_rob_y - map_points[base_num + 1][1])
            back_diff_xy = math.sqrt((world_rob_x - map_points[base_num - 1][0])**2+world_rob_y - map_points[base_num - 1][1])

            if front_diff_xy < back_diff_xy:
                next_num = base_num + 1
            else:
                next_num = base_num - 1

        return  base_num, next_num
