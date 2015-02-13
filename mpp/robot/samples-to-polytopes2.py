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

###############################################################################
### load all samples, which should be investigated (k=1 homotopy class)
###############################################################################
for i in range(0,len(XLarray)):
        [k,h1,h2,h3] = Harray[i]
        kctr[k]+=1
        if k==3:
                XLarrayK.append(XLarray[i])
                XLarrayH.append(Harray[i])

###############################################################################
### get unique H1,H2,H3 values in the samples
###############################################################################
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
#plot = Plotter()
#[X,Y,Z] = projectDataOnto3MainAxes(XLarrayK)
print "samples=",len(XLarrayK)
#[X,Y,Z] = plotDataMainAxes(XLarrayK,[])


XLarrayK = np.array(XLarrayK)
XLarrayK = XLarrayK.reshape(-1, XSPACE_DIMENSION)

###############################################################################
### kpca
###############################################################################
print "kpca"
print XLarrayK.shape

from sklearn.decomposition import PCA, KernelPCA
kpca = KernelPCA(kernel='poly', degree=20)
X_kpca = kpca.fit_transform(XLarrayK)
print X_kpca.shape
print "done"

if DEBUG_CLUSTERH3_PLOT:
        S = kpca.lambdas_
        v = np.sum(S[0:3])/np.sum(S)
        print "[KPCA] variability 3 main axes:",v
        fig=figure(1)
        ax = fig.gca(projection='3d')
        XX = X_kpca[:,1]
        YY = X_kpca[:,2]
        ZZ = X_kpca[:,3]
        ax.scatter(XX,YY,ZZ,marker='o',c='r',s=5)
        #ax.view_init(13,63)
        ax.view_init(2,-58)
        plt.show()
        sys.exit(0)
        for j in range(0,len(H3)):
                Hvalid=np.array(XLarrayH[:,3]==H3[j])
                Xh = XX[Hvalid]
                Yh = YY[Hvalid]
                Zh = ZZ[Hvalid]
                ax.scatter(Xh,Yh,Zh, marker='*',c='b',s=200)
                plt.pause(0.01)

plt.show()
sys.exit(0)
###############################################################################
### obtain h3 clusters by investigating the unique H3 value samples
###############################################################################

## cvxP will contain the convex hull of clusters, i.e. samples with the same h3
## value
hullClustersH3 = []
for j in range(0,len(H3)):
        Hvalid=np.array(XLarrayH[:,3]==H3[j])
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
        if d<0.05:
                h3clusters_tmp.append(i+1)
        else:
                h3clusters.append(h3clusters_tmp)
                h3clusters_tmp = []
                h3clusters_tmp.append(i+1)
        D[i]=np.around(d,3)

if len(h3clusters_tmp)>0:
        h3clusters.append(h3clusters_tmp)

###############################################################################
### merged h3 clusters split, according to their h2 values
###############################################################################

clusterSamples = []
clusterSamplesH2 = []
ctr=0
print "-----------------------------------------------------------------------"
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

        print "[H3] cluster",i,":",h3i,"samples:",curX.shape[1]

        ctr+=curX.shape[1]

print "[H3] -------- total samples:",ctr
print "-----------------------------------------------------------------------"
assert ctr == len(XLarrayK)

###############################################################################
### visualizing [H3] clusters
###############################################################################
if DEBUG_CLUSTERH3_PLOT:
        fig=figure(1)
        ax = fig.gca(projection='3d')
        for i in range(0,len(h3clusters)):
                h3i = h3clusters[i]

                ### put together all h3 clusters in interval h3i
                curX = np.zeros([3,0])
                curH2 = np.zeros([1,0])


                Hvalid = (XLarrayH[:,3]==H3[h3i[0]])
                for j in range(1,len(h3i)):
                        Hvalid |= (XLarrayH[:,3]==H3[h3i[j]])

                XX = X[Hvalid]
                YY = Y[Hvalid]
                ZZ = Z[Hvalid]

                ax.scatter(XX,YY,ZZ, marker='*',c='b',s=200)
                #ax.view_init(18,23)
                ax.view_init(65,130)
                plt.pause(0.01)


sys.exit(0)

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
