import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")

from scipy.cluster.vq import kmeans2,kmeans
from sklearn.cluster import AffinityPropagation
from scipy.spatial import ConvexHull
import pickle
import scipy.io
import numpy as np
import networkx as nx
from mathtools.timer import *
from robot.robotspecifications import *
from robot.displayXspacePCA import *
from mathtools.polytope import *
from mathtools.linalg import *
from mathtools.plotter import *
from robot.xtoplot import *

L = 900

hcStr = str(K_HOMOTOPY_CLASS)
### load XLarray
folder = os.environ["MPP_PATH"]+"mpp-robot/output/kpca"
XLname = folder+"/XL"+hcStr+".dat"
XLarrayFileName = pickle.load( open( XLname, "rb" ) )
XLarray = pickle.load( open( XLarrayFileName, "rb" ) )

XRname = folder+"/XR1.dat"
XRarrayFileName = pickle.load( open( XRname, "rb" ) )
XRarray = pickle.load( open( XRarrayFileName, "rb" ) )

XLarray = np.array(XLarray)
XLarray = XLarray.reshape(-1, XSPACE_DIMENSION)
XRarray = np.array(XRarray)
XRarray = XRarray.reshape(-1, XSPACE_DIMENSION)

folder = os.environ["MPP_PATH"]+"mpp-robot/output/clusters"
xh2name=folder+"/XH2_clustering"+hcStr+".dat"
allClusters = pickle.load( open( xh2name, "rb" ) )

timer = Timer("H2 clustering to polytope")

###############################################################################
### plot cluster
###############################################################################
color="rb"
fig=figure(1)
ax = fig.gca(projection='3d')
G_R = nx.Graph()
clusterCtr=0

for i in range(0,len(allClusters)):
        subClusters = allClusters[i]
        if i<len(allClusters)-1:
                XXc = allClusters[i][0][0][:,0:3]
                XXn = allClusters[i+1][0][0][:,0:3]
                xyzc=np.mean(XXc,axis=0)
                xyzn=np.mean(XXn,axis=0)
                #ax.plot([xyzc[0],xyzn[0]],[xyzc[1],xyzn[1]],zs=[xyzc[2],xyzn[2]],c='k')
                Nc = allClusters[i][0][1]
                Nn = allClusters[i+1][0][1]
                G_R.add_edge(Nc,Nn)
                #axToSphere(ax,0.02,xyzc)

        for j in range(0,len(subClusters)):
                XXc = allClusters[i][j][0][:,0:3]
                xyzc=np.mean(XXc,axis=0)
                allClusters[i][j].append(xyzc)
                #ax.scatter(xyzc[0],xyzc[1],xyzc[2],c='g',s=300,marker='o')
                clusterCtr+=1

                if j < len(subClusters)-1:
                        Nc = allClusters[i][j][1]
                        Nn = allClusters[i][j+1][1]
                        G_R.add_edge(Nc,Nn)
                        XXc = allClusters[i][j][0][:,0:3]
                        XXn = allClusters[i][j+1][0][:,0:3]
                        xyzc=np.mean(XXc,axis=0)
                        xyzn=np.mean(XXn,axis=0)
                        #ax.plot([xyzc[0],xyzn[0]],[xyzc[1],xyzn[1]],zs=[xyzc[2],xyzn[2]],c='k')

                XX = subClusters[j][0][:,0:3]
                xyz=np.mean(XX,axis=1)
                #ax.scatter(XX[:,0],XX[:,1],XX[:,2],c=color[(j+i*len(subClusters))%len(color)],s=10,marker='o')


#pos=nx.spring_layout(G_R)
#nx.draw_networkx_nodes(G_R, pos, node_color='r')
#nx.draw_networkx_edges(G_R, pos, edge_color='b', width=1.0,alpha=0.5)

print "connected components:",nx.number_connected_components(G_R)

#sys.exit(0)
###############################################################################
### cluster -> polytopes
###############################################################################
Aarray=[]
barray=[]
ARKarray=[]
Parray=[]
PHyperarray=[]

G_R = nx.Graph()

for i in range(0,len(allClusters)):

        subClusters = allClusters[i]

        if i<len(allClusters)-1:
                Nc = allClusters[i][0][1]
                Nn = allClusters[i+1][0][1]
        Parray_tmp=[]
        PHyperarray_tmp=[]
        for j in range(0,len(subClusters)):
                I = subClusters[j][0][:,7]
                I = I.astype(int)
                I=np.array(I.flatten()).flatten()
                XLclust = XLarray[I,:]
                XRclust = XRarray[I,:]

                #samples = subClusters[j][0][0:3]

                M = XLclust.shape[0]
                if M<=XSPACE_DIMENSION+1:
                        ## fill it up
                        while M*M+M<XSPACE_DIMENSION:
                                k=int(np.random.uniform(0,M))
                                p=int(np.random.uniform(0,M))
                                XI = XLclust[k,:]+0.5*(XLclust[p,:]-XLclust[k,:])
                                XLclust=np.vstack([XLclust,XI])
                                XI = XRclust[k,:]+0.5*(XRclust[p,:]-XRclust[k,:])
                                XRclust=np.vstack([XRclust,XI])
                                M=XLclust.shape[0]

                        for k in range(0,M):
                                for p in range(0,M):
                                        if XLclust.shape[0]>XSPACE_DIMENSION+1:
                                                break
                                        XI = XLclust[k,:]+0.5*(XLclust[p,:]-XLclust[k,:])
                                        XLclust=np.vstack([XLclust,XI])
                                        XI = XRclust[k,:]+0.5*(XRclust[p,:]-XRclust[k,:])
                                        XRclust=np.vstack([XRclust,XI])
                                        # xspaceDisplay(XI,XI,XI)

                hull = ConvexHull(-XLclust,qhull_options="QJ")
                E=hull.equations
                A=E[:,0:XSPACE_DIMENSION]
                b=np.array((E[:,-1]))

                b=np.expand_dims(b, axis=1)

                #M = XLclust.shape[0]
                #k=int(np.random.uniform(0,M))
                #XI = projectPointOntoPolytopeNdim(XLclust[k,:], A, b)
                #xspaceDisplay(XI,XI,XI)
                #### DEBUG: make sure all xlclust samples are inside A,b
                #for k in range(0,M):
                #        P = np.less_equal(dot(A,XLclust[k,:]),b)
                #        if not P.all():
                #                print "sample",k,"/",M,"not inside polytope!"
                #                print P
                #                sys.exit(0)

                Ah=np.zeros((2*XSPACE_DIMENSION,XSPACE_DIMENSION))
                bh=np.zeros((2*XSPACE_DIMENSION,1))
                #X=np.zeros((XSPACE_DIMENSION,1))

                for p in range(0,XSPACE_DIMENSION):
                        L=min(XLclust[:,p])
                        U=max(XLclust[:,p])
                        Ah[p,p] = 1.0
                        Ah[p+XSPACE_DIMENSION,p] = -1.0
                        bh[p] = U
                        bh[p+XSPACE_DIMENSION] = -L

                Aarray.append(A)
                barray.append(b)

                [Ark,d] = findProjectionMatrixArray(XLclust,XRclust)
                d=np.around(d,5)
                print "cluster [",i,"|",j,"]",clusterCtr,"L->R=",d
                ARKarray.append(Ark)
                P = Polytope(A,b)
                P.setArk(Ark)

                H = Polytope(Ah,bh)
                PHyperarray_tmp.append(H)
                P=P.intersectWithPolytope(H)
                Parray_tmp.append(P)

        Parray.append(Parray_tmp)
        PHyperarray.append(PHyperarray_tmp)
#plt.show()
output_folder = os.environ["MPP_PATH"]+"mpp-robot/output/polytopes"
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

Pname =   output_folder+"/P.dat"
PHypername =   output_folder+"/Phyper.dat"
Gname =   output_folder+"/G.dat"
Aname =   output_folder+"/A.dat"
ARKname = output_folder+"/Ark.dat"
bname =   output_folder+"/b.dat"

#pickle.dump( HVarray, open( Hname, "wb" ) )
pickle.dump( Aarray, open( Aname, "wb" ) )
pickle.dump( barray, open( bname, "wb" ) )
pickle.dump( ARKarray, open( ARKname, "wb" ) )
pickle.dump( Parray, open( Pname, "wb" ) )
pickle.dump( G_R, open( Gname, "wb" ) )
pickle.dump( PHyperarray, open( PHypername, "wb" ) )

timer.stop()
