from math import *

def get_circle_intersection( r1: float, r2: float, d: float):

    """
    Get the intersection points of two circles lying on the x axis.

    First circle lies on the origin. Second circle is d units away from the first circle
    on the x axis.

    Eqn: C1: x^2+y^2=r1^2, C2:(x-d)^2+y^2=r2^2

    Params
    ------
    r1: float
        Radius of first circle.
    r2: float
        Radius of second circle.
    d: float
        Distance between origin of both circles
    x: float
        X coor of points
    y1: float
        Y coor of point above y axis
    y2: float 
        Y coor of point below y axis
    alpha: float
        Angle between line passing through the points and the x axis
    """


    x = (r1**2-r2**2+d**2)/(2*d)    # sub y^2=r2^2-(x-d)^2 into c1
    y1 = sqrt(abs(r1**2 - x**2))    # sub into first circle
    y2 = -y1
    alpha = atan(y1/abs(x))

    return x, y1, y2, alpha

def confine_ang(ang):
    """Confine self.heading to 0 and 360 degrees"""
    ang %= 360
    if ang < 0: ang += 360
    return ang

def is_ang_in_range(ang: float, lower_bound: float, upper_bound: float) -> bool:
    """ 
    Check if an angle is within a range.

    E.g. if ang=27, is_ang_in_range(ang, 0, 90) -> True
        if ang=27, is_ang_in_range(ang, 270, 90) -> True
        if ang-27, is_ang_in_range(ang, 90, 270) -> False
    Parameters
    ----------
    ang: float
        angle to check (from 0-360)
    lower_bound: float
        smaller value (from 0-360)
    upper_bound: float
        larger value (from 0-360)
    """ 

    lower_bound = confine_ang(lower_bound)
    upper_bound = confine_ang(upper_bound)

    if lower_bound < upper_bound:
        return lower_bound <= ang <= upper_bound
    else:
        return (ang >= lower_bound) or (ang <= upper_bound)

# functions 
def get_angular_diff(intendedAngle: float, currentAng: float) -> float: # cw => -ve ccw => +ve 
    """
    Get the differenc between two angles

    Parameters
    ----------
    intendedAngle: float
        Difference from this position
    currentAng: float
        Position you are currently facing
    angDiff: float
        difference between the two positions| +ve: acw, -ve: cw
    """

    angDiff = intendedAngle - currentAng

    angDiff = (360 - angDiff) if angDiff > 180 else (360 + angDiff) if angDiff < -180 else angDiff
    return -angDiff

def calculate_route(r:float, x:float, y:float):
    """
    Calculates the arc angle and length of tangent.

    Parameters
    ----------
    theta:  float
        Angle to turn to relative to the normal. -ve => acw, +ve cw
    tangent:   float
        Distance to travel on the tangent to the arcs. Cm
    """
    r = abs(r)
    print(f"x: {x}, y:{y}")
    if y < r*2: raise ValueError("End point too close")
    if y < 0: raise ValueError("Y should  be infront of car")

    cntr_2_cntr_length = sqrt((abs(x)-2*r)**2+y**2)                             # length of line btw the 2 arc centers
    alpha = degrees(acos(r/(cntr_2_cntr_length/2)))                        # angle between radius and c2c line
    phi = degrees(atan2(y,abs(x)-2*r))                               # angle from c2c line to +ve x axis
    theta = 180 - alpha - phi                                     # angle of tangent to normal
    tangent = (y - 2*r*sin(radians(theta))) / cos(radians(theta))  # length of tangent

    # if x is negative, turning direction is opposite
    if x < 0:
        return -theta, tangent
    else:
        return theta, tangent