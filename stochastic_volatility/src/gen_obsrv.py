from random import *
from math import *

x = 0
for i in range(10):
    y = exp(x/2.0)
    print 'x: {0} y: {1}'.format(x, y)
    x = 0.91*x + gauss(0,1)
