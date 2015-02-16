import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")

from scipy.cluster.vq import kmeans2,kmeans
from sklearn.cluster import AffinityPropagation
from scipy.spatial import ConvexHull
import pickle
import scipy.io
import numpy as np
from mathtools.timer import *
from robot.robotspecifications import *
from robot.displayXspacePCA import *
from mathtools.polytope import *
from mathtools.linalg import *
from mathtools.plotter import *

folder = os.environ["MPP_PATH"]+"mpp-robot/output"
datafolder = "/h3_0_01_h2_0_01_h1_0_01"
#datafolder = "/h3_0_02_h2_0_02_h1_0_02"
#datafolder = "/h3_0_01_h2_0_005_h1_0_005"
#datafolder = "/h3_0_01_h2_0_002_h1_0_002"
#datafolder = ""

xfolder = folder+"/xspace"+datafolder
XLname =   xfolder+"/xsamplesL.dat"
XRname =   xfolder+"/xsamplesR.dat"
Hname =    xfolder+"/hsamples.dat"
Harray = pickle.load( open( Hname, "rb" ) )
XLarray = pickle.load( open( XLname, "rb" ) )

timer = Timer("KPCA projection")

K_homotopy_class = K_HOMOTOPY_CLASS
kctr = [0,0,0,0]

XLarrayK = []
XLarrayH = []

###############################################################################
### load all samples, which should be investigated (k=1 homotopy class)
###############################################################################
for i in range(0,len(XLarray)):
        [k,h1,h2,h3] = Harray[i]
        kctr[k]+=1
        if k==K_homotopy_class:
                XLarrayK.append(XLarray[i])
                XLarrayH.append([k,h1,h2,h3,i])

print "samples=",len(XLarrayK),"homotopy class",K_HOMOTOPY_CLASS

XLarrayK = np.array(XLarrayK)
XLarrayK = XLarrayK.reshape(-1, XSPACE_DIMENSION)

###############################################################################
### kpca
###############################################################################
print XLarrayK.shape
from sklearn.decomposition import PCA, KernelPCA
kpca = KernelPCA(kernel='linear', degree=40)
X_kpca = kpca.fit_transform(XLarrayK)
print X_kpca.shape

S = kpca.lambdas_
v = np.sum(S[0:3])/np.sum(S)
print "[KPCA] variability 3 main axes:",v

XLkpca = np.zeros((XLarrayK.shape[0],8))
for i in range(0,XLarrayK.shape[0]):
        [k,h1,h2,h3,index] = XLarrayH[i]
        xx = X_kpca[i,0]
        yy = X_kpca[i,1]
        zz = X_kpca[i,2]
        XLkpca[i,0]=xx
        XLkpca[i,1]=yy
        XLkpca[i,2]=zz
        XLkpca[i,3]=k
        XLkpca[i,4]=h1
        XLkpca[i,5]=h2
        XLkpca[i,6]=h3
        XLkpca[i,7]=index

output_folder = os.environ["MPP_PATH"]+"mpp-robot/output/kpca"
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

xfile=output_folder+"/X"+str(K_homotopy_class)+".dat"
pickle.dump( XLkpca, open( xfile, "wb" ) )
xlfile=output_folder+"/XL"+str(K_homotopy_class)+".dat"
pickle.dump( XLname, open( xlfile, "wb" ) )
xrfile=output_folder+"/XR"+str(K_homotopy_class)+".dat"
pickle.dump( XRname, open( xrfile, "wb" ) )

timer.stop()
