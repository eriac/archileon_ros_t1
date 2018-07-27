def search_value_tube(way_points, world_tube_x, world_tube_y):
    import math

    base_num = 0
    list_max_num = len(way_points) - 1
    tmp_points_x = way_points[0][0]
    tmp_points_y = way_points[0][1]
    diff_x = world_tube_x - tmp_points_x
    diff_y = world_tube_y - tmp_points_y
    base_diff_xy = math.sqrt(diff_x**2 + diff_y**2)

    for i in range(list_max_num):
        tmp_points_x = way_points[i][0]
        tmp_points_y = way_points[i][1]
        diff_x = world_tube_x - tmp_points_x
        diff_y = world_tube_y - tmp_points_y
        tmp_diff_xy = math.sqrt(diff_x**2 + diff_y**2)

        if tmp_diff_xy < base_diff_xy:
            base_num = i
            base_diff_xy = math.sqrt(diff_x**2 + diff_y**2)

    return base_num


def get_dist_tube_to_points(forward_x, forward_y, world_tube_x, world_tube_y):
    import math

    diff_x = world_tube_x - forward_x
    diff_y = world_tube_y - forward_y
    diff_xy = math.sqrt(diff_x**2 + diff_y**2)

    return diff_xy
