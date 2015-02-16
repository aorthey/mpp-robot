from    dionysus    import Rips, PairwiseDistances, StaticPersistence, Filtration, points_file, \
                           ExplicitDistances, data_dim_cmp
from    sys         import argv, exit
import  time

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

folder = os.environ["MPP_PATH"]+"mpp-robot/output"
datafolder = "/h3_0_01_h2_0_01_h1_0_01"
#datafolder = "/h3_0_02_h2_0_02_h1_0_02"
#datafolder = "/h3_0_01_h2_0_005_h1_0_005"
#datafolder = "/h3_0_01_h2_0_002_h1_0_002"
#datafolder = ""

xfolder = folder+"/xspace"+datafolder
XLname =   xfolder+"/xsamplesL.dat"
Hname =    xfolder+"/hsamples.dat"
Harray = pickle.load( open( Hname, "rb" ) )
XLarray = pickle.load( open( XLname, "rb" ) )
XLarray = np.array(XLarray)
XLarray = XLarray.reshape(-1, XSPACE_DIMENSION)

XLarrayK = []
XLarrayH = []
for i in range(0,len(XLarray)):
        [k,h1,h2,h3] = Harray[i]
        if k==K_HOMOTOPY_CLASS:
                XLarrayK.append(XLarray[i])
                XLarrayH.append([k,h1,h2,h3,i])
print "homotopy class",K_HOMOTOPY_CLASS,"samples",len(XLarrayK)

XLarrayK = np.array(XLarrayK)
points = XLarrayK.tolist()
distances = PairwiseDistances(points)
# distances = ExplicitDistances(distances)           # speeds up generation of the Rips complex at the expense of memory usage
rips = Rips(distances)
print time.asctime(), "Rips initialized"

skeleton=3
maxParam = 50
simplices = Filtration()

rips.generate(skeleton, maxParam, simplices.append)

print time.asctime(), "Generated complex: %d simplices" % len(simplices)

# While this step is unnecessary (Filtration below can be passed rips.cmp), 
# it greatly speeds up the running times
for s in simplices: s.data = rips.eval(s)
print time.asctime(), simplices[0], '...', simplices[-1]

simplices.sort(data_dim_cmp)             # could be rips.cmp if s.data for s in simplices is not set
print time.asctime(), "Set up filtration"

p = StaticPersistence(simplices)
print time.asctime(), "Initialized StaticPersistence"

p.pair_simplices()
print time.asctime(), "Simplices paired"

print "Outputting persistence diagram"
smap = p.make_simplex_map(simplices)
for i in p:
    if i.sign():
        b = smap[i]

        if b.dimension() >= skeleton: continue

        if i.unpaired():
            print b.dimension(), b.data, "inf"
            continue

        d = smap[i.pair()]
        print b.dimension(), b.data, d.data
