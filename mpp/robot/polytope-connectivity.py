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
G_R = robot_folder+"/graph_robot.dat"

G_R = pickle.load( open( G_R, "rb" ) )

#plot=Plotter()
#plot.graphLayout(G_R)
pos=nx.spring_layout(G_R)
nx.draw_networkx_nodes(G_R, pos, node_color='r')
nx.draw_networkx_edges(G_R, pos, edge_color='b', width=1.0,alpha=0.5)
plt.show()

