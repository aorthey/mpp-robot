import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")

import pickle
import scipy.io
import numpy as np
from timeit import default_timer as timer
from robot.displayXspacePCA import *

L = 900

folder = os.environ["MPP_PATH"]+"mpp-robot/output"
#xfolder = folder+"/xspace/h1_0_01_h2_0_001_h3_0_001"
#datafolder = "/h1_0_01_h2_0_005_h3_0_005"
datafolder = "/h1_0_01_h2_0_002_h3_0_002"

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

XLarray = pickle.load( open( XLname, "rb" ) )
#XRarray = pickle.load( open( XRname, "rb" ) )
#XMarray = pickle.load( open( XMname, "rb" ) )
Harray = pickle.load( open( Hname, "rb" ) )
[Npts, VSTACK_DELTA, heights] = pickle.load( open( HeadName, "rb" ) )

Nsamples=len(XLarray)

i = 0
h3cur = 0
HarraySameH3 = []
HarraySubsets = []
while i < Nsamples:

        [k,h1,h2,h3] = Harray[i]
        if h3>h3cur:
                HarraySubsets.append([HarraySameH3,h3])
                HarraySameH3=[]
                HarraySameH3.append(XLarray[i])
                h3cur = h3
        else:
                HarraySameH3.append(XLarray[i])
        i=i+1

### compute PCA on the middle array
M = int(math.floor(len(HarraySubsets)*0.5))
print M
PCaxes = getMainPCAaxes(HarraySubsets[M][0],HarraySubsets[M][1],Npts)

for i in range(0,len(HarraySubsets)):
        Hsame = HarraySubsets[i][0]
        h3 = HarraySubsets[i][1]
        istr = str(i).zfill(4)
        fnameFig = outputfolder+"/xspacePCA"+istr+".png"
        print h3,"with",len(Hsame),"samples projected onto PC and plotted to",fnameFig
        plotDataMainAxes(Hsame,PCaxes,h3,Npts,fnameFig)

ffmpegstr = "ffmpeg -y -framerate 10 -start_number 0000 -i "+outputfolder+"/xspacePCA%04d.png -pix_fmt yuv420p "+outputfolder+"/out.mp4"
vlcstr = "vlc "+outputfolder+"/out.mp4"
os.system(ffmpegstr)
os.system(vlcstr)
