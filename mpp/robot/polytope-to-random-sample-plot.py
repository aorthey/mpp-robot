import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-path-planner/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")

from timeit import default_timer as timer
import numpy as np
import cvxpy as cvx
import pickle
import networkx as nx
from mathtools.plotter import Plotter,rotFromRPY
from numpy import inf,array,zeros
from cvxpy import *
from mathtools.util import *
from mathtools.linalg import *
from mathtools.plotter import *
from robot.xtoplot import *

from matplotlib.patches import Circle 


robot_folder = os.environ["MPP_PATH"]+"mpp-robot/output/polytopes"

Phypername =   robot_folder+"/Phyper.dat"
Aname =   robot_folder+"/A.dat"
ARKname = robot_folder+"/Ark.dat"
bname =   robot_folder+"/b.dat"
Pname =   robot_folder+"/P.dat"
G_R = robot_folder+"/G.dat"

G_R = pickle.load( open( G_R, "rb" ) )
P = pickle.load( open( Pname, "rb" ) )
ARK = pickle.load( open( ARKname, "rb" ) )
Hyper = pickle.load( open( Phypername, "rb" ) )

N = len(P)
fig=figure(1)
ax = fig.gca()
ctr = 0

## draw random sample for each convex set
np.random.seed(seed=9)
for i in range(0,N):

        Parray=P[i]
        darray=[]
        dstart = 0

        if i<N-1:
                Pn=P[i+1]
                d = distancePolytopePolytope(Parray[0],Pn[0])
                print i,"->",i+1,"=",d

        for j in range(0,len(Parray)):
                if j<len(Parray)-1:
                        d = distancePolytopePolytope(P[i][j],P[i][j+1])
                        print " --",j,"->",j+1,"=",d
                #XL = P[i][j].getRandomSample()
                Hsample = Hyper[i][j].getRandomSampleHyperrectangle()
                A = P[i][j].A
                b = P[i][j].b
                Hsample = np.array(Hsample)
                XL = projectPointOntoPolytopeNdim(Hsample, A, b)
                XL = projectPointOntoPolytopeNdim(XL, A, b)
                vv = np.less_equal(dot(A,XL),b)
                #if vv.all():
                #        print "ok"
                #else:
                #        print "nok"
                #print "Hsample",Hsample.shape
                #print "Xsample",XL.shape
                #XL = P[i][j].getRandomSampleWithKnownHyperrectangle(Hyper[i][j])
                Ar = P[i][j].Atmp
                XR = np.dot(Ar,XL)
                xspaceDisplay(XL,XL,XR)

        #print "["+str(i)+"]",dstart,"|",darray

ax.axis('equal') 
#plot=Plotter()
#plot.graphLayout(G_R)
#pos=nx.spring_layout(G_R)
#nx.draw_networkx_nodes(G_R, pos, node_color='r')
#nx.draw_networkx_edges(G_R, pos, edge_color='b', width=1.0,alpha=0.5)
plt.show()

