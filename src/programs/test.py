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

    if lower_bound < upper_bound:
        return lower_bound <= ang <= upper_bound
    else:
        return (ang <= lower_bound) or (ang >= upper_bound)
ang = 0    
print(is_ang_in_range(ang, 5, 355))
print(ang)