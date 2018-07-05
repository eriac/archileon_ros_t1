#coding: UTF-8 
def calLine(center_x, center_y, p1_x, p1_y, p2_x, p2_y): 
    #sympyで円と直線の交点 
    #http://gologius.hatenadiary.com/entry/2014/12/23/193018 
    import sympy.geometry as sg

    radius = 0.1050
    center = sg.Point(center_x,center_y) 
    circle = sg.Circle(center, radius) 
     
    line = sg.Segment(sg.Point(p1_x, p1_y), sg.Point(p2_x, p2_y)) 
 
    result = sg.intersection(circle, line) 
    return result
 
 
def calCurve(center1_x, center1_y, center2_x, center2_y): 
    import sympy.geometry as sg 
    tube_radius = 0.1050
    way_points_radius = 0.450
 
    center = sg.Point(center1_x,center1_y) 
    circle1 = sg.Circle(center, tube_radius) 
 
    center = sg.Point(center2_x,center2_y) 
    circle2 = sg.Circle(center, way_points_radius) 

    result = sg.intersection(circle1,circle2)
    return result

def cal(tube_rot_axis_x, tube_rot_axis_y, tube_x, tube_y):
    if 0 <= tube_x <= 1.0 and tube_y <=0.50:
        result = calLine(
            center_x=tube_rot_axis_x, center_y=tube_rot_axis_y, 
            p1_x=0.0, p1_y=0.050, 
            p2_x = 1.0, p2_y=0.050,
        )
        print("AAAA")

    elif 0 <= tube_x <= 1.0 and tube_y >0.50:
        result = calLine(
            center_x=tube_rot_axis_x, center_y=tube_rot_axis_y, 
            p1_x=0.0, p1_y=0.950, 
            p2_x = 1.0, p2_y=0.950,
        )
        print("BBBB")

    elif tube_x > 1.0:
        result = calCurve(
            center1_x=tube_rot_axis_x, center1_y=tube_rot_axis_y, 
            center2_x=1.0, center2_y=-0.50
        )
        print("CCCC")

    elif tube_x < 0.0:
        result = calCurve(
            center1_x=tube_rot_axis_x, center1_y=tube_rot_axis_y, 
            center2_x=0.0, center2_y=-0.50
        )
        print("DDDD")

    return result