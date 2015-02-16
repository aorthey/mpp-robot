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

Xname = folder+"/X"+str(K_HOMOTOPY_CLASS)+".dat"

Xkpca = pickle.load( open( Xname, "rb" ) )

## when to merge two clusters
DIST_INTERNAL_CLUSTER=0.03

fig=figure(1)
ax = fig.gca(projection='3d')
X = Xkpca[:,0]
Y = Xkpca[:,1]
Z = Xkpca[:,2]
#ax.scatter(X,Y,Z,marker='o',c='r',s=5)

M = X.shape[0]
H3=[]
H2=[]
H1=[]
for j in range(0,M):
        [k,h1,h2,h3,index]=Xkpca[j,3:]
        H1.append(h1)
        H2.append(h2)
        H3.append(h3)

H1=np.unique(H1)
H2=np.unique(H2)
H3=np.unique(H3)

ax.view_init(94,-63)

hullClustersH3 = []
for j in range(0,len(H3)):
        Hvalid=np.array(Xkpca[:,6]==H3[j])
        HvalidH2 = (Xkpca[:,5]==H2[len(H2)-1])
        for k in reversed(range(0,len(H2))):
                HvalidH2 |= (Xkpca[:,5]==H2[k])
                HvalidInternal = Hvalid & HvalidH2
                if len(X[HvalidInternal])>len(H2)/2:
                        break

        Hvalid = Hvalid & HvalidH2

        Xh2 = X[Hvalid]
        Yh2 = Y[Hvalid]
        Zh2 = Z[Hvalid]
        XX = np.array([Xh2,Yh2,Zh2])
        hull = ConvexHull(XX.T)
        E=hull.equations[0::2]
        Ah = np.array(E[0:,0:3])
        bh = np.zeros((len(Ah),1))
        for k in range(0,len(Ah)):
                bh[k] = -E[k,3]
        P = Polytope(Ah,bh)
        hullClustersH3.append(P)

D = np.zeros((len(hullClustersH3)-1,1))

###############################################################################
### compute distance between clusters, and merge nearby clusters together
###############################################################################
h3clusters = []
h3clusters_tmp = [0]
for i in range(0,len(hullClustersH3)-1):
        cp = hullClustersH3[i]
        cpp = hullClustersH3[i+1]
        d = distancePolytopePolytope(cp,cpp)
        if d<DIST_INTERNAL_CLUSTER:
                h3clusters_tmp.append(i+1)
        else:
                h3clusters.append(h3clusters_tmp)
                h3clusters_tmp = []
                h3clusters_tmp.append(i+1)
        D[i]=np.around(d,3)

if len(h3clusters_tmp)>0:
        h3clusters.append(h3clusters_tmp)

for i in range(0,len(h3clusters)):
        cp = hullClustersH3[i]

clusterSamples = []
clusterSamplesH2 = []

ctr=0
XHclusters=[]
print "-----------------------------------------------------------------------"
for i in range(0,len(h3clusters)):
        h3i = h3clusters[i]

        ### put together all h3 clusters in interval h3i
        curX = np.zeros([3,0])
        curH2 = np.zeros([1,0])

        Hvalid = (Xkpca[:,6]==H3[h3i[0]])
        for j in range(1,len(h3i)):
                Hvalid |= (Xkpca[:,6]==H3[h3i[j]])

        Xh2 = X[Hvalid]
        Yh2 = Y[Hvalid]
        Zh2 = Z[Hvalid]

        HH = Xkpca[Hvalid,:]
        lh3 = H3[h3i[0]]
        uh3 = H3[h3i[len(h3i)-1]]
        XHclusters.append([HH,lh3,uh3])

        print "[H3] cluster",i,"h3=",lh3,uh3,":",h3i,"samples:",len(Xh2)
        ctr+=len(Xh2)


print "[H3] -------- total samples:",ctr,"/",M
assert ctr==M
print "-----------------------------------------------------------------------"

output_folder = os.environ["MPP_PATH"]+"mpp-robot/output/clusters"
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

dname=output_folder+"/XH3_clustering"+str(K_HOMOTOPY_CLASS)+".dat"
pickle.dump( XHclusters, open( dname, "wb" ) )

if DEBUG_CLUSTERH3_PLOT:
        fig=figure(1)
        ax = fig.gca(projection='3d')
        color="rb"
        for i in range(0,len(h3clusters)):
                h3i = h3clusters[i]

                ### put together all h3 clusters in interval h3i
                curX = np.zeros([3,0])
                curH2 = np.zeros([1,0])

                Hvalid = (Xkpca[:,6]==H3[h3i[0]])
                for j in range(1,len(h3i)):
                        Hvalid |= (Xkpca[:,6]==H3[h3i[j]])

                XX = X[Hvalid]
                YY = Y[Hvalid]
                ZZ = Z[Hvalid]

                ax.scatter(XX,YY,ZZ, marker='o',c=color[i%len(color)],s=10)
                ax.view_init(52,-102)
                ax.set_xlabel('X_0')
                ax.set_ylabel('X_1')
                ax.set_zlabel('X_2')

        plt.savefig('kpca.svg')
        plt.show()
