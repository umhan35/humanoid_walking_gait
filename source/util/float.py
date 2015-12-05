

def is_float_equal(f, x):
    return abs(f - x) < 1e-10


def is_float_greater_than_0(f):
    # f !=0 for case that 'f' is 0.00000000000000000010>0 but actually is 0
    return (not is_float_equal(f, 0)) and f > 0


def is_float_less_than_0(f):
    # f !=0 for case that 'f' is 0.00000000000000000010>0 but actually is 0
    return (not is_float_equal(f, 0)) and f < 0