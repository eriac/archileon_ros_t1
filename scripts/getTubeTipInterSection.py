def cal(tube_rot_axis_position_x, tube_rot_axis_position_y, tube_rot_axis_radius, tube_tip_base_point_x, tube_tip_base_point_y, tube_tip_adj_point_x,tube_tip_adj_point_y):
    #sympyで円と直線の交点
    import sympy
    center_x = tube_rot_axis_position_x
    center_y = tube_rot_axis_position_y

    tube_tip_base_point_x = tube_tip_base_point_x
    tube_tip_base_point_y = tube_tip_base_point_y

    tube_tip_adj_point_x = tube_tip_adj_point_x
    tube_tip_adj_point_y = tube_tip_adj_point_y

    print("center")
    print(center_x)
    print(center_y)
    print("tube_tip_base_point")
    print(tube_tip_base_point_x)
    print(tube_tip_base_point_y)
    print("tube_tip_adj_point")
    print(tube_tip_adj_point_x)
    print(tube_tip_adj_point_y)
    print()

    sympy.var('xm ym r x1 y1 x2 y2')
    ci=sympy.Circle(sympy.Point(xm,ym),r)
    ln = sympy.Line(sympy.Point(x1, y1), sympy.Point(x2, y2))
    result=ci.intersection(ln)
    input={xm:center_x, ym:center_y, r:tube_rot_axis_radius, x1:tube_tip_base_point_x, y1:tube_tip_base_point_y, x2:tube_tip_adj_point_x, y2:tube_tip_adj_point_y}
    ans_1=result[0].subs(input)
    ans_2=result[1].subs(input)
    print("answer")
    print(ans_1)
    print(ans_2)
    return ans_1, ans_2
