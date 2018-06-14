def getDistance(map_points, ):

    from sympy import *
    var('x1 y1 x2 y2 x3 y3 x4 y4')
    ln12 = Line(Point(x1, y1), Point(x2, y2))
    ln34 = Line(Point(x3, y3), Point(x4, y4))
    ls=ln12.intersection(ln34)

    input={x1:0,y1:0, x2:1, y2:1, x3:0, y3:1, x4:1, y4:0};
    x = ls[0].x.subs(input)
    y = ls[0].y.subs(input))
