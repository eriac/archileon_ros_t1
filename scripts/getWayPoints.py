# coding: UTF-8
def read_rob():
    import os
    point_list = []

    base = os.path.dirname(os.path.abspath(__file__))
    data = "/robot.txt"
    data_path = base + data

    f = open(data_path, "r")
    line = f.readline()
    point_list.append(line.strip())

    while line:
        line = f.readline()
        point_list.append(line.strip())
    line_length = len(point_list) - 1

    way_points = []
    for i in range(line_length):
        xy_list = []
        x = float(point_list[i][1:point_list[i].index(",")]) - 0.5
        y = float(point_list[i][point_list[i].index(
            ",")+1: point_list[i].index("]")])+0.5
        xy_list.append(x)
        xy_list.append(y)
        way_points.append(xy_list)
    f.close()
    return way_points


def read_bl_tube_points():
    import os
    point_list = []

    base = os.path.dirname(os.path.abspath(__file__))
    data = "/nozzle_in.txt"
    data_path = base + data

    f = open(data_path, "r")
    line = f.readline()
    point_list.append(line.strip())

    while line:
        line = f.readline()
        point_list.append(line.strip())
    line_length = len(point_list) - 1

    way_points = []
    for i in range(line_length):
        xy_list = []
        x = float(point_list[i][1:point_list[i].index(",")]) - 0.5
        y = float(point_list[i][point_list[i].index(
            ",")+1: point_list[i].index("]")])+0.5
        xy_list.append(x)
        xy_list.append(y)
        way_points.append(xy_list)
    f.close()
    return way_points


def read_br_tube_points():
    import os
    point_list = []

    base = os.path.dirname(os.path.abspath(__file__))
    data = "/nozzle_out.txt"
    data_path = base + data

    f = open(data_path, "r")
    line = f.readline()
    point_list.append(line.strip())

    while line:
        line = f.readline()
        point_list.append(line.strip())
    line_length = len(point_list) - 1

    way_points = []
    for i in range(line_length):
        xy_list = []
        x = float(point_list[i][1:point_list[i].index(",")]) - 0.5
        y = float(point_list[i][point_list[i].index(
            ",")+1: point_list[i].index("]")])+0.5
        xy_list.append(x)
        xy_list.append(y)
        way_points.append(xy_list)
    f.close()
    return way_points
