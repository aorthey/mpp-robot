import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-path-planner/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")

from scipy.spatial import ConvexHull
import random as rnd
sys.path.append("..")
from mathtools.polytope import *
from mathtools.linalg import *
import scipy.spatial as spatial

from pylab import *
from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection 
import matplotlib.pyplot as plt 
from timeit import default_timer as timer
from robot.htox import *
import numpy as np
import pickle
from math import fabs

#folder="xspace"
env_folder = os.environ["MPP_PATH"]+"mpp-robot/output/xspace"

env_folder+="/h1_0_01_h2_0_01_h3_0_01"
#env_folder+="/h1_0_01_h2_0_005_h3_0_005"
#env_folder+="/h1_0_01_h2_0_001_h3_0_001"
XLname   = env_folder+"/xsamplesL.dat"
XRname   = env_folder+"/xsamplesR.dat"
XMname   = env_folder+"/xsamplesM.dat"
Hname    = env_folder+"/hsamples.dat"
HeadName = env_folder+"/headersamples.dat"

start = timer()

print XLname
XLarray = pickle.load( open( XLname, "rb" ) )
Harray = pickle.load( open( Hname, "rb" ) )
[Npts, VSTACK_DELTA, heights] = pickle.load( open( HeadName, "rb" ) )

M = len(XLarray)
N = len(XLarray[0])

xK = np.zeros((M,N))
hK = np.zeros((M,N,4))

def dmetric(xi,xj):
        return np.linalg.norm(xi-xj)

i=0
xnghbrj = 0
xcurj = 0
while xnghbrj is not nan:
        xcur = XLarray[xcurj]
        xK[i,:] = XLarray[xcurj].T
        print i,"/",M
        i=i+1

        minD = 10000

        xnghbr = nan
        xnghbrj = nan

        if xcur is nan:
                break

        for j in range(0,xcurj):
                xn = XLarray[j]
                if xn is not nan:
                        d = dmetric(xcur,xn)
                        if d<minD:
                                minD = d
                                xnghbr = xn
                                xnghbrj = j
        for j in range(xcurj+1,M):
                xn = XLarray[j]
                if xn is not nan:
                        d = dmetric(xcur,xn)
                        if d<minD:
                                minD = d
                                xnghbr = xn
                                xnghbrj = j

        XLarray[xcurj]=nan
        xcurj = xnghbrj

#xK = np.sort(xK,axis=1)
#Nk = 2
#xK = xK[xK[:,Nk].argsort()]
#hK = hK[xK[:,Nk].argsort()]

startX = 0
f, axray = plt.subplots(16, sharex=True, sharey=True)

t = arange(0,len(xK))

for i in range(0,len(axray)):
        axray[i].plot(t,xK[:,i+startX], '-or', markersize=2)

end = timer()

ts= np.around(end - start,2)
tm= int(ts/60)
tms = np.around(ts - tm*60,2)

print "================================================================"
print "Time elapsed for building free space decomposition:"
print "================="
print tm,"m",tms,"s (",ts,"s)"
print "================================================================"
plt.show()
