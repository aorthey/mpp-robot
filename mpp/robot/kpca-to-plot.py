import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")

from scipy.cluster.vq import kmeans2,kmeans
from sklearn.cluster import AffinityPropagation
from scipy.spatial import ConvexHull
import pickle
import scipy.io
import numpy as np
from robot.robotspecifications import *
from robot.displayXspacePCA import *
from mathtools.polytope import *
from mathtools.linalg import *
from mathtools.plotter import *

folder = os.environ["MPP_PATH"]+"mpp-robot/output/kpca"

Xname = folder+"/X1.dat"

X = pickle.load( open( Xname, "rb" ) )

fig=figure(1)
ax = fig.gca(projection='3d')
XX = X[:,0]
YY = X[:,1]
ZZ = X[:,2]
ax.scatter(XX,YY,ZZ,marker='o',c='r',s=5)

M = X.shape[0]
H3=[]
H2=[]
H1=[]
for j in range(0,M):
        [k,h1,h2,h3,index]=X[j,3:]
        H1.append(h1)
        H2.append(h2)
        H3.append(h3)

H1=np.unique(H1)
H2=np.unique(H2)
H3=np.unique(H3)
ax.view_init(117,-83)
for j in range(0,len(H2)):
        Hvalid=np.array(X[:,5]==H2[j])
        Xh = X[Hvalid,0]
        Yh = X[Hvalid,1]
        Zh = X[Hvalid,2]
        ax.scatter(Xh,Yh,Zh, marker='*',c='b',s=200)
        plt.pause(0.01)

plt.show()
sys.exit(0)
