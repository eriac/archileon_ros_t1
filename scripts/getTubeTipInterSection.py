def cal(rob_tube_rot_axis_position, rob_tube_radius, tube_base_point, tube_adj_point):
    #sympyで円と直線の交点
    from sympy import *
    center_x = rob_tube_rot_axis_position[0]
    center_y = rob_tube_rot_axis_position[1]

    tube_base_point_x = tube_base_point[0]
    tube_base_point_y = tube_base_point[1]

    tube_adj_point_x = tube_adj_point[0]
    tube_adj_point_y = tube_adj_point[1]

    var('xm ym r x1 y1 x2 y2')
    ci=Circle(Point(xm,ym),r)
    ln = Line(Point(x1, y1), Point(x2, y2))
    result=ci.intersection(ln)
    input={xm:center_x, ym:center_y, r:rob_tube_radius, x1:tube_base_point_x, y1:tube_base_point_y, x2:tube_adj_point_x, y2:tube_adj_point_y}
    ans_1=result[0].subs(input)
    ans_2=result[1].subs(input)
    return ans_1, ans_2
