# coding: UTF-8
def read_rob():
    import os
    point_list = []

    base = os.path.dirname(os.path.abspath(__file__))
    rob_data = "/robot.txt"
    rob_org = "/rob_origin.txt"
    rob_data_path = base + rob_data
    rob_org_path = base + rob_org

    try:
        f = open(rob_data_path, "r")
        line = f.readline()
        point_list.append(line.strip())
    except:
        print("READ_ERROR")

    try:
        g = open(rob_org_path, "r")
        rob_org_x = float(g.readline().strip())
        rob_org_y = float(g.readline().strip())
    except:
        print("READ_ERROR")

    while line:
        line = f.readline()
        point_list.append(line.strip())
    line_length = len(point_list) - 1

    way_points = []
    for i in range(line_length):
        xy_list = []
        x = float(point_list[i][1:point_list[i].index(",")]
                  ) - 0.5 + rob_org_x
        y = float(point_list[i][point_list[i].index(
            ",")+1: point_list[i].index("]")]) + rob_org_y
        xy_list.append(x)
        xy_list.append(y)
        way_points.append(xy_list)
    f.close()
    g.close()
    return way_points


def read_bl_tube_points():
    import os
    point_list = []

    base = os.path.dirname(os.path.abspath(__file__))
    rob_data = "/nozzle_in.txt"
    rob_org = "/rob_origin.txt"
    nozzle_in_data_path = base + rob_data
    rob_org_path = base + rob_org

    try:
        f = open(nozzle_in_data_path, "r")
        line = f.readline()
        point_list.append(line.strip())

    except:
        print("READ_ERROR")
    try:
        g = open(rob_org_path, "r")
        rob_org_x = float(g.readline().strip())
        rob_org_y = float(g.readline().strip())
    except:
        print("READ_ERROR")

    while line:
        line = f.readline()
        point_list.append(line.strip())
    line_length = len(point_list) - 1

    way_points = []
    for i in range(line_length):
        xy_list = []
        x = float(point_list[i][1:point_list[i].index(",")]
                  ) - 0.5 + rob_org_x
        y = float(point_list[i][point_list[i].index(
            ",")+1: point_list[i].index("]")]) + rob_org_y
        xy_list.append(x)
        xy_list.append(y)
        way_points.append(xy_list)
    f.close()
    g.close()
    return way_points


def read_br_tube_points():
    import os
    point_list = []

    base = os.path.dirname(os.path.abspath(__file__))
    rob_data = "/nozzle_out.txt"
    rob_org = "/rob_origin.txt"
    nozzle_out_data_path = base + rob_data
    rob_org_path = base + rob_org

    try:
        f = open(nozzle_out_data_path, "r")
        line = f.readline()
        point_list.append(line.strip())
    except:
        print("READ_ERROR")

    try:
        g = open(rob_org_path, "r")
        rob_org_x = float(g.readline().strip())
        rob_org_y = float(g.readline().strip())
    except:
        print("READ_ERROR")

    while line:
        line = f.readline()
        point_list.append(line.strip())
    line_length = len(point_list) - 1

    way_points = []
    for i in range(line_length):
        xy_list = []
        x = float(point_list[i][1:point_list[i].index(",")]
                  ) - 0.5 + rob_org_x
        y = float(point_list[i][point_list[i].index(
            ",")+1: point_list[i].index("]")]) + rob_org_y
        xy_list.append(x)
        xy_list.append(y)
        way_points.append(xy_list)
    f.close()
    g.close()
    return way_points
