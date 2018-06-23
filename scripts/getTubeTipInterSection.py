#coding: UTF-8
def cal(center_x, center_y, radius, p1_x, p1_y, p2_x, p2_y):
    #sympyで円と直線の交点
    #http://gologius.hatenadiary.com/entry/2014/12/23/193018
    import sympy.geometry as sg

    center = sg.Point(center_x,center_y)
    radius = radius
    circle = sg.Circle(center, radius)
    segment = sg.Segment(sg.Point(p1_x, p1_y), sg.Point(p2_x, p2_y))

    result = sg.intersection(circle, seg0ment)
    return result
