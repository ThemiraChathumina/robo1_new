import math

def list_to_matrix(lst):
    matrix = []
    for i in range(64):
        row = lst[i*64:(i+1)*64]
        matrix.append(row)
    return matrix

def map_angle(angle):
    if angle == 180 or angle == -180:
        return 180
    elif angle >= 0:
        return (360 - angle) % 180 + 180
    else:
        return (-angle) % 180

def add_subtract_angles(angle1, angle2):
    return (angle1 + angle2) % 360

def angle_difference(angle1, angle2):
    diff = abs(angle1 - angle2) % 360
    x = min(diff, 360 - diff)
    if 0 <= angle2 < 180 and 180 <= angle1 < 360:
        return -1 * x
    elif 0 <= angle1 < 180 and 180 <= angle2 < 360:
        return x
    elif angle1 > angle2:
        return x
    else:
        return -1*x

def circle_circle_intersection(circle1, circle2):
    x1, y1, r1 = circle1
    x2, y2, r2 = circle2
    d = math.hypot(x2 - x1, y2 - y1)
    if d > r1 + r2:
        print("no solution")
        return None
    if d < abs(r1 - r2):
        print("no solution")
        return None
    if d == 0 and r1 == r2:
        print("no solution")
        return None
    a = (r1**2 - r2**2 + d**2) / (2 * d)
    h = math.sqrt(r1**2 - a**2)
    xm = x1 + a * (x2 - x1) / d
    ym = y1 + a * (y2 - y1) / d
    xs1 = xm + h * (y2 - y1) / d
    xs2 = xm - h * (y2 - y1) / d
    ys1 = ym - h * (x2 - x1) / d
    ys2 = ym + h * (x2 - x1) / d
    # return (xs1, ys1), (xs2, ys2)
    if ys1 > ys2:
        return xs1, ys1
    else:
        return xs2, ys2


#python function to calculate the angle of a given point in a triangle formed by three points
def getAngle(x1, y1, x2, y2, x3, y3):
    #calculating the length of the sides of the triangle
    a = math.sqrt((x2-x1)**2 + (y2-y1)**2)
    b = math.sqrt((x3-x2)**2 + (y3-y2)**2)
    c = math.sqrt((x3-x1)**2 + (y3-y1)**2)

    #calculating the angle
    angle = math.acos((a**2 + b**2 - c**2)/(2*a*b))
    return math.degrees(angle)





import math

def calc_angle(x1, y1, x2, y2, ox, oy):
    dx1 = x1 - ox
    dy1 = y1 - oy
    dx2 = x2 - ox
    dy2 = y2 - oy
    try:
        angle = math.atan2(dy2, dx2) - math.atan2(dy1, dx1)
    except ZeroDivisionError:
        angle = 0.0
    angle = math.degrees(angle)
    return angle



def getAngles(x,y):
    x1, y1, r1 = 0, 0, 7.5
    x2, y2, r2 = x, y, 7.5
    x,y = circle_circle_intersection((x1, y1, r1), (x2, y2, r2))
    alpha = abs(getAngle(x2,y2,x1,y1,x,y)) + abs(getAngle(x2,0,0,0,x2,y2))
    beta = abs(180 - getAngle(x2,y2,x,y,x1,y1))
    gamma = abs(180 - getAngle(x2,-100,x2,y2,x,y))
    delta = abs(90 - gamma)
    return (alpha, beta, gamma, delta)



print(math.degrees(math.atan2(2, 1)))



# import threading
#
#
# def run_functions_concurrently(functions):
#     """Run a list of functions concurrently using threads."""
#     threads = []
#     for function in functions:
#         thread = threading.Thread(target=function[0], args=function[1])
#         threads.append(thread)
#         thread.start()
#
#     for thread in threads:
#         thread.join()
#
# def func1(x, y):
#     print(f"Function 1: x={x}, y={y}")
#
# def func2(x, y, z):
#     print(f"Function 2: x={x}, y={y}, z={z}")
#
# def func3():
#     print("Function 3")
#
# # List of functions to run concurrently
# functions = [
#     (func1, (1, 2)),
#     (func2, (3, 4, 5)),
#     (func3, ())
# ]
#
# # Run the functions concurrently
# run_functions_concurrently(functions)
