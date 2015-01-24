from timeit import default_timer as timer
import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")

import random as rnd
from math import cos,sin,tan,pi,asin,acos,atan2,atan
import pickle
import numpy as np
from robot.robotspecifications import *
from robot.htox import *

from pylab import *
from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection 
import matplotlib.pyplot as plt 


khomotopy=1

### optimize over h3, plot

h3 = 1.2
h1 = 0.4
h2 = 1.0
start = timer()

X = []
while h2 > -1:
        h2 = h2-0.0001
        [xL,xM,xR,q,theta] = hspace2xspace(khomotopy,h1,h2,h3)
        if xL is not None:
                X.append(xL)
#h1 = 0.7
#h3 = 1.7
#while h3 > 0:
#        [xL,xM,xR,q,theta] = hspace2xspace(k,h1,h2,h3)
#        h3 = h3-0.0001
#        if xL is not None:
#                X.append(xL)

fig = [0,0,0,0]
axray = [0,0,0,0]
X = np.array(X)

print "sampled",len(X),"points, h=",h1,h2,h3
if len(X)>0:
        fig, axray = plt.subplots(8,5, sharex=True, sharey=True)
        for j in range(0,5):
                t = arange(0,len(X))
                for i in range(0,8):
                        axray[i][j].plot(t,X[:,j*8 + i], '-or', markersize=2)
                        axray[i][j].plot([0.0,len(X)],[0.0,0.0], '-k')
                        axray[i][j].text(0.0, 0.0, str(j*8+i), size=20, va="center",\
                                        ha="center", backgroundcolor='#CCCCCC')
                        for k in range(0,len(X)-1):
                                xk = X[k,j*8+i]
                                xkk = X[k+1,j*8+i]
                                if abs(xk-xkk) > 0.01:
                                        axray[i][j].plot([k,k],[xk-0.15,xk+0.15],'-ok',linewidth=1)



        plt.subplots_adjust(hspace = .1, wspace = 0.001)
        end = timer()
        ts= np.around(end - start,2)
        tm= int(ts/60)
        tms = np.around(ts - tm*60,2)

        print "================================================================"
        print "Time elapsed for plotting h3 variation"
        print "================="
        print tm,"m",tms,"s (",ts,"s)"
        print "================================================================"

        plt.show()
