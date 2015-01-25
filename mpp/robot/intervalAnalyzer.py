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


khomotopy=3

fig, axray = plt.subplots(2,1, sharex=True, sharey=True)

XH1 = []
XLarray = []
XRarray = []
H1 = []
ctr= 0

ctrH3 = 0
h3 = 1.8

while h3 > 0.8:
        h3 = h3-0.1
        ctrH1 = 0
        print "h3=",h3,"samples=",len(XLarray)
        h1 = 2.0
        while h1 > 0:
                h1 = h1-0.1
                h2 = -0.3
                while h2 < 0.3:
                        h2 = h2+0.01
                        [xL,xM,xR,q,theta] = hspace2xspace(khomotopy,h1,h2,h3)
                        if xL is not None:
                                XLarray.append(xL)
                                XRarray.append(xR)
                                if len(XLarray)>1:
                                        xc = XLarray[-1]
                                        xl = XLarray[-2]
                                        if np.linalg.norm(xc-xl) > 0.001:
                                                XH1.append(1)
                                        else:
                                                XH1.append(0)
                                else:
                                        XH1.append(0)
                                H1.append(h1)
                                ctr=ctr+1
                        else:
                                XH1.append(None)
                                H1.append(None)

X = np.array(XH1)
H = np.array(H1)
print "h=",khomotopy,h1,h2,h3,"->",ctr,"samples/",len(XH1)
for i in range(0,len(XLarray)):
        xL = XLarray[i]
        xR = XRarray[i]
        xspaceDisplay(xL,None,xR)

print "done"

t = arange(0,len(X))
axray[0].plot(t, X, '-or', markersize=3)
axray[1].plot(t, H, '-or', markersize=3)
plt.show()
sys.exit(0)
