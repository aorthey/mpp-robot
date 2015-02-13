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

robot_folder = os.environ["MPP_PATH"]+"mpp-robot/output/polytopes"

Aname =   robot_folder+"/A.dat"
ARKname = robot_folder+"/Ark.dat"
bname =   robot_folder+"/b.dat"

Aflat = pickle.load( open( Aname, "rb" ) )
Ark = pickle.load( open( ARKname, "rb" ) )
bflat = pickle.load( open( bname, "rb" ) )


P = []
for i in range(0,len(Aflat)):
        P.append(Polytope(Aflat[i],bflat[i]))

N = len(P)
D = np.zeros((N,N))
print "loaded",N,"polytopes"
G_R = nx.Graph()
for i in range(0,N):
        for j in range(i+1,N):
                d=distancePolytopePolytope(P[i],P[j])
                D[i,j]=D[j,i]=d
                if d<=0.01:
                        G_R.add_edge(i,j)

pickle.dump( G_R, open( robot_folder+"/graph_robot.dat", "wb" ) )
pickle.dump( D, open( robot_folder+"/distance_matrix.dat", "wb" ) )
print "wrote connectivity graph and distance matrix to",robot_folder
