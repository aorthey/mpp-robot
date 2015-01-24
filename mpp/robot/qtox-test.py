from timeit import default_timer as timer
import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")
from robot.htox import *
from robot.qtox import *

q=np.array((0,0,0,0,0))
[xL,xM,xR]=qtox(q)
xspacePlot(xL,xM,xR,show=True)
## test mirroing
q=-q
[xL,xM,xR]=qtox(q)
xspacePlot(xL,xM,xR,show=True)

q=np.array((-1.1,1.1,0.0,0.0,0.0))
[xL,xM,xR]=qtox(q)
xspacePlot(xL,xM,xR,show=True)

## test mirroing
q=-q
[xL,xM,xR]=qtox(q)
xspacePlot(xL,xM,xR,show=True)

q=np.array((-1.0,1.0,0.0,-0.8,0.0))
[xL,xM,xR]=qtox(q)
xspacePlot(xL,xM,xR,show=True)

## test mirroing
q=-q
[xL,xM,xR]=qtox(q)
xspacePlot(xL,xM,xR,show=True)


