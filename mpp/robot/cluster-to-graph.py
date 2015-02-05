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

robot_folder = os.environ["MPP_PATH"]+"mpp-robot/output/clusters"

Pname = robot_folder+"/polytopes.dat"
Dname = robot_folder+"/D.dat"
P = pickle.load( open( Pname, "rb" ) )
D = pickle.load( open( Dname, "rb" ) )

G_R = nx.Graph()
N = len(D)

fig=figure(1)
ax = fig.gca(projection='3d')
X=[]
for i in range(0,N):
        X.append(P[i].xyz)

X=np.array(X)

ax.scatter(X[:,0],X[:,1],X[:,2], marker='*',c='b',s=200)

for i in range(0,N):
        for j in range(i+1,N):
                if D[i,j]<=0.6:
                        ax.plot([X[i,0],X[j,0]],[X[i,1],X[j,1]],[X[i,2],X[j,2]], '-or', markersize=2)
                        G_R.add_edge(i,j)


print "connected components:",nx.number_connected_components(G_R)
plt.show()

#plot=Plotter()
#plot.graphLayout(G_R)
pos=nx.spring_layout(G_R)
nx.draw_networkx_nodes(G_R, pos, node_color='r')
nx.draw_networkx_edges(G_R, pos, edge_color='b', width=1.0,alpha=0.5)
print "connected components:",nx.number_connected_components(G_R)
