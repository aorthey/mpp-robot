from timeit import default_timer as timer
import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")
from robot.htox import *

h1 = 0.35
h2 = -0.7
h3 = 1.11

while h2 < 0.5:
        [q,theta]=htoq(1,h1,h2,h3)
        h2 = h2+0.01
        if q is not None:
                [xL,xR,xM]=qtox(q)
                if xL is not None:
                        print "SUCCESS:",q,theta
                        xspacePlot(xL,xM,xR,show=True)

