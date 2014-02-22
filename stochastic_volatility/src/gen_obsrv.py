from random import *
from math import *

fx = open('data_x.txt','w')
fy = open('data_y.txt','w')
fo = open('obsrv.txt','w')

x = 0
for i in range(10):
    x = 0.91*x + gauss(0,1)
    y = gauss(0,exp(x/2.0))
    print 'x: {0} y: {1}'.format(x, y)
    fx.write(str(x)+'\n')
    fy.write(str(y)+'\n')
    fo.write(str(y)+'\n')

fx.close()
fy.close()
fo.close()
