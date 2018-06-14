def read():
    f = open("/mnt/hgfs/ROS_ENV/catkin_ws/src/archileon_ros_t1/scripts/way_points.txt", "r")
    line = f.readline()

    line_length = 1713
    point_list=[]
    while line:
        line = f.readline()
        point_list.append(line)

    new_list = []
    for i in range(line_length):
        new_list.append(point_list[i].strip().strip(" ,"))

    way_points=[]
    for i in range(line_length):
        xy_list =[]
        x = float(new_list[i][1:new_list[i].index(",")])*0.001
        y = float(new_list[i][new_list[i].index(",")+1: new_list[i].index("]")])*0.001
        xy_list.append(x)
        xy_list.append(y)
        way_points.append(xy_list)
    f.close()
    return way_points
