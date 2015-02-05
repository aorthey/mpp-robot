import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")

from scipy.cluster.vq import kmeans2,kmeans
from sklearn.cluster import AffinityPropagation
from scipy.spatial import ConvexHull
import pickle
import scipy.io
import numpy as np
from timeit import default_timer as timer
from robot.robotspecifications import *
from robot.displayXspacePCA import *
from mathtools.polytope import *
from mathtools.linalg import *
from mathtools.plotter import *

L = 900

folder = os.environ["MPP_PATH"]+"mpp-robot/output"
datafolder = "/h3_0_01_h2_0_01_h1_0_01"
#datafolder = "/h3_0_02_h2_0_02_h1_0_02"
#datafolder = "/h3_0_01_h2_0_005_h1_0_005"
#datafolder = "/h3_0_01_h2_0_002_h1_0_002"
#datafolder = ""

xfolder = folder+"/xspace"+datafolder

XLname =   xfolder+"/xsamplesL.dat"
XRname =   xfolder+"/xsamplesR.dat"
XMname =   xfolder+"/xsamplesM.dat"
Hname =    xfolder+"/hsamples.dat"
HeadName = xfolder+"/headersamples.dat"

start = timer()
Harray = pickle.load( open( Hname, "rb" ) )
[Npts, VSTACK_DELTA, heights] = pickle.load( open( HeadName, "rb" ) )

kctr = [0,0,0,0]
XRarray = pickle.load( open( XRname, "rb" ) )
XLarray = pickle.load( open( XLname, "rb" ) )

XLarrayK = []
XLarrayH = []

for i in range(0,len(XLarray)):
        [k,h1,h2,h3] = Harray[i]
        kctr[k]+=1
        if k==1:
                XLarrayK.append(XLarray[i])
                XLarrayH.append(Harray[i])

H1=[]
H2=[]
H3=[]
for i in range(0,len(XLarrayK)):
        [k,h1,h2,h3] = XLarrayH[i]
        H1.append(h1)
        H2.append(h2)
        H3.append(h3)

XLarrayH=np.array(XLarrayH)
H1=np.unique(H1)
H2=np.unique(H2)
H3=np.unique(H3)

print "dimensions=",len(H2),len(H3)
plot = Plotter()
#[X,Y,Z] = projectDataOnto3MainAxes(XLarrayK)
print "samples=",len(XLarrayK)
[X,Y,Z] = plotDataMainAxes(XLarrayK,[])

###############################################################################
### obtain h3 clusters
###############################################################################
cvxP = []
for j in range(0,len(H3)):
        Hvalid=np.array(XLarrayH[:,3]==H3[j])
        Xh2 = X[Hvalid]
        Yh2 = Y[Hvalid]
        Zh2 = Z[Hvalid]
        XX = np.array([Xh2,Yh2,Zh2])
        [xp,yp,zp,S]=getMainPCAaxes(XX)
        hull = ConvexHull(XX.T)
        E=hull.equations[0::2]
        Ah = np.array(E[0:,0:3])
        bh = np.zeros((len(Ah),1))
        for k in range(0,len(Ah)):
                bh[k] = -E[k,3]
        P = Polytope(Ah,bh)
        cvxP.append(P)

D = np.zeros((len(cvxP)-1,1))

h3clusters = []
h3clusters_tmp = [0]
for i in range(0,len(cvxP)-1):
        cp = cvxP[i]
        cpp = cvxP[i+1]
        d = distancePolytopePolytope(cp,cpp)
        if d<0.05:
                h3clusters_tmp.append(i+1)
        else:
                h3clusters.append(h3clusters_tmp)
                h3clusters_tmp = []
                h3clusters_tmp.append(i+1)
        D[i]=np.around(d,3)
        print np.around(d,2)

if len(h3clusters_tmp)>0:
        h3clusters.append(h3clusters_tmp)

print "h3 clusters=",len(h3clusters),h3clusters
ctr=0

clusterSamples = []
clusterSamplesH2 = []
for i in range(0,len(h3clusters)):
        h3i = h3clusters[i]

        ### put together all h3 clusters in interval h3i
        curX = np.zeros([3,0])
        curH2 = np.zeros([1,0])
        for j in range(0,len(h3i)):
                Hvalid = (XLarrayH[:,3]==H3[h3i[j]])
                Xh2 = X[Hvalid]
                Yh2 = Y[Hvalid]
                Zh2 = Z[Hvalid]
                XX = np.array([Xh2,Yh2,Zh2])
                curX = np.append(curX,XX,axis=1)
                curH2 = np.append(curH2,XLarrayH[Hvalid,2])

        clusterSamplesH2.append(curH2)
        clusterSamples.append(curX)

        print "samples:",curX.shape[1]
        ctr+=curX.shape[1]

assert ctr == len(XLarrayK)

###############################################################################
### obtain h2 clusters inside the h3 clusters
###############################################################################
clusterCtr = 0
clustersCtrArray = []

allClusters=[]
for i in range(0,len(clusterSamples)):
        internalClusterCtr = []
        ch2 = clusterSamplesH2[i]
        H2=np.unique(ch2)

        samples = clusterSamples[i]
        cvxP = []

        clusters = []
        clustersI = []
        singleClust = np.zeros([3,0])
        singleClustI= []

        svalue = np.zeros((len(H2),1))
        for j in range(0,len(H2)):
                Hvalid=np.array(ch2==H2[j])
                Xh2 = samples[0,Hvalid]
                Yh2 = samples[1,Hvalid]
                Zh2 = samples[2,Hvalid]
                XX = np.array([Xh2,Yh2,Zh2])

                singleClust = np.append(singleClust,XX,axis=1)

                if singleClust.shape[1]>2:
                        [xp,yp,zp,S]=getMainPCAaxes(singleClust)
                        ss = float(S[2])/float(np.sum(S))
                        svalue[j]=ss
                        if ss>0.05:
                                ## linearity violated, break
                                if singleClust.shape[1]>3:
                                        clusters.append(singleClust)
                                        clustersI.append(singleClustI)
                                singleClust = np.zeros([3,0])
                                singleClustI=[j]
                        else:
                                singleClustI.append(j)
                else:
                        singleClustI.append(j)

        if singleClust.shape[1]>3:
                clusters.append(singleClust)
                clustersI.append(singleClustI)

        print "cluster",i,"has",samples.shape[1],"samples and",len(clusters),"clusters"

        ctr=0
        for j in range(0,len(clusters)):
                print "--- subcluster",j,":",clustersI[j],"has",clusters[j].shape[1],"samples"
                ctr+=clusters[j].shape[1]
                internalClusterCtr.append(clusters[j].shape[1])
                allClusters.append(clusters[j])
        print "-----------------",ctr,"samples"

        clustersCtrArray.append(internalClusterCtr)
        if not len(clusters)>0:
                sys.exit(0)

print "total number of clusters:",len([item for sublist in clustersCtrArray for item in sublist])

cvxP=[]
fig=figure(1)
ax = fig.gca(projection='3d')

ctrSamples=0
for i in range(0,len(allClusters)):
        C = allClusters[i]
        ctrSamples+=C.shape[1]
        hull = ConvexHull(C.T)
        E=hull.equations[0::2]
        Ah = np.array(E[0:,0:3])
        bh = np.zeros((len(Ah),1))
        for k in range(0,len(Ah)):
                bh[k] = -E[k,3]
        xyz=np.mean(C,axis=1)
        P = Polytope(Ah,bh,xyz)
        cvxP.append(P)
        ax.scatter(C[0,:],C[1,:],C[2,:], marker='*',c='b',s=200)
        #plt.pause(0.01)

print "samples",ctrSamples

### visualize clusters

print "clusters to cvx hull. done."

D = np.zeros((len(cvxP),len(cvxP)))
for i in range(0,len(cvxP)):
        for j in range(i+1,len(cvxP)):
                cp = cvxP[i]
                cpp = cvxP[j]
                d = distancePolytopePolytope(cp,cpp)
                D[i,j]=D[j,i]=d

print np.around(D,2)

output_folder = os.environ["MPP_PATH"]+"mpp-robot/output/clusters"
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

dname=output_folder+"/D.dat"
pname=output_folder+"/polytopes.dat"
pickle.dump( D, open( dname, "wb" ) )
pickle.dump( cvxP, open( pname, "wb" ) )

sys.exit(0)
