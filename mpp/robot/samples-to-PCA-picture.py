import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")

import pickle
import scipy.io
import numpy as np
from timeit import default_timer as timer
from robot.displayXspacePCA import *
from robot.robotspecifications import *

L = 900

folder = os.environ["MPP_PATH"]+"mpp-robot/output"
datafolder = "/h3_0_01_h2_0_01_h1_0_01"
#datafolder = "/h3_0_01_h2_0_005_h1_0_005"
#datafolder = "/h3_0_01_h2_0_001_h1_0_001"
#datafolder = ""

xfolder = folder+"/xspace"+datafolder

XLname =   xfolder+"/xsamplesL.dat"
XRname =   xfolder+"/xsamplesR.dat"
XMname =   xfolder+"/xsamplesM.dat"
Hname =    xfolder+"/hsamples.dat"
HeadName = xfolder+"/headersamples.dat"
outputfolder=folder+"/pca"+datafolder

if not os.path.exists(outputfolder):
    os.makedirs(outputfolder)

start = timer()

Harray = pickle.load( open( Hname, "rb" ) )
[Npts, VSTACK_DELTA, heights] = pickle.load( open( HeadName, "rb" ) )

kctr = [0,0,0,0]
XLarray = pickle.load( open( XLname, "rb" ) )
XLarrayK = []
XLarrayH = []
for i in range(0,len(XLarray)):
        [k,h1,h2,h3] = Harray[i]
        kctr[k]+=1
        if k==1:
                XLarrayK.append(XLarray[i])
                XLarrayH.append(Harray[i])

H=[]
for i in range(0,len(XLarrayK)):
        [k,h1,h2,h3] = XLarrayH[i]
        H.append(h3)

hk = 3
XLarrayH=np.array(XLarrayH)
H=np.unique(H)

print "homotopy class counter:",kctr
### compute PCA on the middle array
fnameFig = outputfolder+"/completePCA.png"

[X,Y,Z]=plotDataMainAxes(XLarrayK,fnameFig)

if X is not None:
        print len(XLarrayK),"samples projected onto PC and plotted to",fnameFig
        fig=figure(1)
        ax = fig.gca(projection='3d')
        ax.view_init(-3, 33)
        i = int(len(H)*0.5)
        for i in range(0,len(H)):
                Xh = X[XLarrayH[:,hk]==H[i]]
                Yh = Y[XLarrayH[:,hk]==H[i]]
                Zh = Z[XLarrayH[:,hk]==H[i]]
                print len(Xh)
                ax.scatter(Xh,Yh,Zh,marker='*',c='b',s=200)
                plt.pause(0.01)
