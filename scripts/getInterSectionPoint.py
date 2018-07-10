#coding: UTF-8 
def calLine(center_x, center_y, p1_x, p1_y, p2_x, p2_y): 
    #sympyで円と直線の交点 
    #http://gologius.hatenadiary.com/entry/2014/12/23/193018 
    import sympy.geometry as sg

    tube_radius = 0.1050
    center = sg.Point(center_x,center_y) 
    circle = sg.Circle(center, tube_radius) 
     
    line = sg.Segment(sg.Point(p1_x, p1_y), sg.Point(p2_x, p2_y)) 
    result = sg.intersection(circle, line)

    return result
 
 