
def float_list_reset_to_0s(list_a):

    diff_list = list(list_a)
    for i, element in enumerate(diff_list):
        diff_list[i] = 0

    return diff_list


def float_list_plus(list_a, list_b):
    # list_b + list_a

    diff_list = list(list_a)
    for i, element in enumerate(list_b):
        diff_list[i] += element

    return diff_list


def float_list_difference(list_a, list_b):
    # list_b - list_a

    diff_list = list(list_a)
    for i, element in enumerate(list_b):
        diff_list[i] -= element

    return diff_list


def float_list_divide(list_a, x):
    diff_list = list(list_a)
    for i, element in enumerate(diff_list):
        diff_list[i] = element / x

    return diff_list


def float_list_multiply(list_a, x):
    diff_list = list(list_a)
    for i, element in enumerate(diff_list):
        diff_list[i] = element * x

    return diff_list
