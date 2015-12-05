from math import degrees, radians


def to_degrees(radians_list):
    converted = list(radians_list)
    for i, r in enumerate(radians_list):
        converted[i] = degrees(r)
    return tuple(converted)


def to_radians(degree_list):
    converted = list(degree_list)
    for i, d in enumerate(degree_list):
        converted[i] = radians(d)
    return tuple(converted)


def angle_range(start, end, step=1):
    """
    Extends the built-in range function.
    Its returned values includes both start and end.
    It can also range from a smaller number to a bigger one.
    """
    if start < end:
        return range(start, end+1, step)
    elif start > end:
        return reversed(range(end, start+1, step))
    else:
        raise NotImplementedError