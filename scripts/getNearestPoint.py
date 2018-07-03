def search_value(way_points, world_rob_x, world_rob_y):
    import math

    base_num = 0
    tmp_points_x = way_points[0][0]
    tmp_points_y = way_points[0][1]
    diff_x = world_rob_x - tmp_points_x
    diff_y = world_rob_y - tmp_points_y
    base_diff_xy = math.sqrt(diff_x**2 + diff_y**2)
    list_max_num = len(way_points) - 1

    for i in range(list_max_num):
        tmp_points_x = way_points[i][0]
        tmp_points_y = way_points[i][1]
        diff_x = world_rob_x - tmp_points_x
        diff_y = world_rob_y - tmp_points_y
        tmp_diff_xy = math.sqrt(diff_x**2 + diff_y**2)

        if tmp_diff_xy < base_diff_xy:
            base_num = i
            base_diff_xy = math.sqrt(diff_x**2 + diff_y**2)

    forward_num = base_num + 1
    return  way_points[base_num], way_points[forward_num]


def search_value_tube(way_points, world_rob_x, world_rob_y):
    import math

    tmp_points_x = way_points[1][0]
    tmp_points_y = way_points[1][1]
    diff_x = world_rob_x - tmp_points_x
    diff_y = world_rob_y - tmp_points_y
    base_diff_xy = math.sqrt(diff_x**2 + diff_y**2)
    base_num = 0
    list_max_num = len(way_points)

    for i in range(list_max_num):
        tmp_points_x = way_points[i][0]
        tmp_points_y = way_points[i][1]
        diff_x = world_rob_x - tmp_points_x
        diff_y = world_rob_y - tmp_points_y
        tmp_diff_xy = math.sqrt(diff_x**2 + diff_y**2)

        if tmp_diff_xy < base_diff_xy:
            base_num = i
            base_diff_xy = math.sqrt(diff_x**2 + diff_y**2)
        else:
            if base_num is list_max_num:
                next_num = base_num
            else:
                next_num = base_num - 1

    return  base_num, next_num
