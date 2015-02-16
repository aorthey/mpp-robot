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

Aname =   robot_folder+"/A.dat"
ARKname = robot_folder+"/Ark.dat"
bname =   robot_folder+"/b.dat"
Pname =   robot_folder+"/P.dat"
G_R = robot_folder+"/G.dat"

G_R = pickle.load( open( G_R, "rb" ) )
P = pickle.load( open( Pname, "rb" ) )
ARK = pickle.load( open( ARKname, "rb" ) )

N = len(P)
fig=figure(1)
ax = fig.gca()
ctr = 0
for i in range(0,N):
        Parray=P[i]
        darray=[]
        dstart = 0
        if i<N-1:
                Pc=P[i][0]
                Pn=P[i+1][0]
                #dstart=np.around(distancePolytopePolytope(Pc,Pn),2)
                ax.plot([0,0],[i,i+1],'-k',zorder=1,linewidth=4)

        for j in range(0,len(Parray)):
                XL = P[i][j].getRandomSample()
                #XL = P[i][j].xyz
                #Ar = ARK[ctr]
                Ar = P[i][j].Atmp
                XR = np.dot(Ar,XL)
                xspaceDisplay(XL,XL,XR)
                ax.add_patch(Circle((j,i), 0.4, fc='g',zorder=5))

        for j in range(0,len(Parray)-1):
                #Pc=Parray[j]
                #Pn=Parray[j+1]
                #d=np.around(distancePolytopePolytope(Pc,Pn),2)
                #darray.append(d)
                ax.plot([j,j+1],[i,i],'-k',zorder=1,linewidth=4)
        ctr+=1

        print "["+str(i)+"]",dstart,"|",darray

ax.axis('equal') 
#plot=Plotter()
#plot.graphLayout(G_R)
#pos=nx.spring_layout(G_R)
#nx.draw_networkx_nodes(G_R, pos, node_color='r')
#nx.draw_networkx_edges(G_R, pos, edge_color='b', width=1.0,alpha=0.5)
plt.show()

