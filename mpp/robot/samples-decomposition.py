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
outputfolder=folder+"/pca"+datafolder

if not os.path.exists(outputfolder):
    os.makedirs(outputfolder)

start = timer()
omem = memory_usage_psutil()

Harray = pickle.load( open( Hname, "rb" ) )
[Npts, VSTACK_DELTA, heights] = pickle.load( open( HeadName, "rb" ) )

kctr = [0,0,0,0]
XRarray = pickle.load( open( XRname, "rb" ) )

XLarray = pickle.load( open( XLname, "rb" ) )
XLarrayK1 = []
XLarrayK3 = []
for i in range(0,len(XLarray)):
        [k,h1,h2,h3] = Harray[i]
        kctr[k]+=1
        if k==1:
                XLarrayK1.append(XLarray[i])
        if k==3:
                XLarrayK3.append(XLarray[i])

print "homotopy class counter:",kctr
### compute PCA on the middle array
fnameFig = outputfolder+"/completePCA.png"
print len(XLarray),"samples projected onto PC and plotted to",fnameFig
[X1,X2,X3] = projectDataOnto3MainAxes(XLarray)
X = np.array([X1,X2,X3])

XLarray = np.array(XLarray)
XLarray = XLarray.reshape(-1, XSPACE_DIMENSION)
XRarray = np.array(XRarray)
XRarray = XRarray.reshape(-1, XSPACE_DIMENSION)

#A = AffinityPropagation()
#Y = A.fit_predict(np.array(X.T))
from scipy.cluster.hierarchy import dendrogram
from scipy.cluster.hierarchy import linkage

fig=figure(1)
fig.clf()
ax = fig.gca(projection='3d')
colors = "bgrcmykw"

k_kmeans = 50
K=kmeans2(X.T,k_kmeans)
labels = K[1]
Aarray = []
ARKarray = []
barray = []

for i in range(0,k_kmeans):

        Xk = X[:,labels==i]
        color = str(i/float(k_kmeans))
        #ax.scatter(Xk[0],Xk[1],Xk[2],c=colors[i%len(colors)])
        if Xk.shape[1]>0:
                XLk = XLarray[labels==i,:]
                A=np.zeros((2*XSPACE_DIMENSION,XSPACE_DIMENSION))
                b=np.zeros((2*XSPACE_DIMENSION,1))
                for p in range(0,XSPACE_DIMENSION):
                        L=min(XLk[:,p])
                        U=max(XLk[:,p])
                        A[p,p] = 1.0
                        A[p+XSPACE_DIMENSION,p] = -1.0
                        b[p] = U
                        b[p+XSPACE_DIMENSION] = -L


                XRk = XRarray[labels==i,:]
                ##find flat conversion matrix, A*x = xr
                [Ark,d] = findProjectionMatrixArray(XLk,XRk)
                #if d > 0.001:
                        #print "projection XL -> XR is not linear",d
                        #sys.exit(0)
                print "cluster",i,"/",k_kmeans,"->",XLk.shape[0],"samples,L->R=",d
                Aarray.append(A)
                barray.append(b)
                ARKarray.append(Ark)

end = timer()
ts= np.around(end - start,2)

output_folder = os.environ["MPP_PATH"]+"mpp-robot/output/polytopes"
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

#Hname =   output_folder+"/H.dat"
Aname =   output_folder+"/A.dat"
ARKname = output_folder+"/Ark.dat"
bname =   output_folder+"/b.dat"

#pickle.dump( HVarray, open( Hname, "wb" ) )
pickle.dump( Aarray, open( Aname, "wb" ) )
pickle.dump( barray, open( bname, "wb" ) )
pickle.dump( ARKarray, open( ARKname, "wb" ) )
print "================================================================"
print "Time elapsed for computing polytopes flats "
print "================="
print ts,"s"
print "================================================================"
print len(Aarray),"polytopes have a memory footprint of",np.around(memory_usage_psutil()-omem,2),"MB"
print "================================================================"
print "data written to",output_folder
print "================================================================"
#ax.view_init(-174,-139)
#plt.show()
