# RPrint 3d printer controller for Raspberry Pi #
# Author: Oehrly, 2017                          #
#
# this file contains some mathematical functions
# used in different places


def sign(a):  # return the sign of number a
    if a > 0:
        return 1
    elif a < 0:
        return -1
    else:
        return 0
