import math

t1 = math.degrees(math.asin(0.71))
print(t1)
# math.acos(t1)


def tan(degree):
    r = math.radians(degree)
    # print('r', r)
    return math.tan(r)

# 0.05/tan(30)
0.05/tan(60)


def old():
    def sin(degree):
        r = math.radians(degree)
        print('r', r)
        return math.sin(r)

    def cos(degree):
        r = math.radians(degree)
        return math.cos(r)

    # x0=0.1
    # x1=0.1+sin(30)*0.8
    # x2=x1-sin(50)*0.7
    # Ex = x2+sin(20)
    #
    # y0=0
    # y1 = sin(60)*0.8
    # y2 = y1+sin(40)*0.7
    # Ey=y2+sin(70)
    #
    # a='---'

    def half(a,b):
        return (a+b) / 2

    # print(a,x0, y0)
    # print(half(x0,x1), half(y0,y1))
    # print(a,x1, y1)
    # print(half(x1,x2), half(y1,y2))
    # print(a,x2, y2)
    # print(half(x2,Ex), half(y2,Ey))
    # print(a,Ex, Ey)

    # 0.3+0.23+
    print(0.07/sin(60))
    print(0.08/sin(30))
    print(sin(60))