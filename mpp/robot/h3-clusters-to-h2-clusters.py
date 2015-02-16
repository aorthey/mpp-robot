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

L = 900

hcStr = str(K_HOMOTOPY_CLASS)
### load XLarray
folder = os.environ["MPP_PATH"]+"mpp-robot/output/kpca"
XLname = folder+"/XL"+hcStr+".dat"
XLarrayFileName = pickle.load( open( XLname, "rb" ) )
XLarray = pickle.load( open( XLarrayFileName, "rb" ) )

XRname = folder+"/XR"+hcStr+".dat"
XRarrayFileName = pickle.load( open( XRname, "rb" ) )
XRarray = pickle.load( open( XRarrayFileName, "rb" ) )

XLarray=np.array(XLarray)
XRarray=np.array(XRarray)

### load clusters according to h3
folder = os.environ["MPP_PATH"]+"mpp-robot/output/clusters"
xh3name=folder+"/XH3_clustering"+hcStr+".dat"
H3clusters = pickle.load( open( xh3name, "rb" ) )

timer = Timer("clustering H2")
###############################################################################
### obtain h2 clusters inside the h3 clusters
###############################################################################
allClusters=[]
clusterCtr = 0

XLarray = np.array(XLarray)
XLarray = XLarray.reshape(-1, XSPACE_DIMENSION)
XRarray = np.array(XRarray)
XRarray = XRarray.reshape(-1, XSPACE_DIMENSION)

for i in range(0,len(H3clusters)):
        samples = H3clusters[i][0]
        lh3 = H3clusters[i][1]
        uh3 = H3clusters[i][2]
        H2 = samples[:,5]
        H2=np.unique(H2)
        M = len(samples)
        print "---------------------------------------------------------------"
        print "[H3] h3=["+str(lh3)+","+str(uh3)+"] samples=",M
        if M<10:
                print " -- cluster["+str(clusterCtr)+"] has",M,"samples"
                allClusters.append([[H3clusters[i][0],clusterCtr,lh3,uh3,H2[0],H2[len(H2)-1]]])
                clusterCtr+=1
                continue

        clusters = []
        ctr=0
        singleClust = np.zeros([0,8])
        while ctr<len(samples):
                d=0.000
                dold = d
                while d<0.0001 and ctr<len(samples):
                        singleClustOld = singleClust
                        #singleClust = np.append(singleClust,XX,axis=1)
                        XX = np.matrix(samples[ctr,:])
                        singleClust = np.append(np.matrix(singleClust),XX,axis=0)
                        I = np.array(singleClust[:,7]).flatten()
                        I = I.astype(int).flatten()
                        XLclust = XLarray[I,:]
                        XRclust = XRarray[I,:]
                        dold = d
                        [Ark,d] = findProjectionMatrixArray(XLclust,XRclust)
                        ctr=ctr+1
                        if singleClust.shape[0]>XSPACE_DIMENSION+2:
                                break

                clusters.append(singleClustOld)
                singleClust = np.matrix(samples[ctr-1,:])

        allClusters_tmp=[]
        for j in range(0,len(clusters)):
                print " -- cluster["+str(clusterCtr)+"] has",clusters[j].shape[0],"samples"
                allClusters_tmp.append([clusters[j],clusterCtr,lh3,uh3,H2[0],H2[len(H2)-1]])
                clusterCtr+=1
        allClusters.append(allClusters_tmp)

xh2name=folder+"/XH2_clustering"+hcStr+".dat"
pickle.dump( allClusters, open( xh2name, "wb" ) )
timer.stop()
