from timeit import default_timer as timer
###############################################################################
import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")

import random as rnd
from math import cos,sin,tan,pi,asin,acos,atan2,atan
import pickle
import numpy as np
from robot.robotspecifications import *
from robot.htox import *
from robot.folderNameFromHvalues import *

from pylab import *
from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection 
import matplotlib.pyplot as plt 

def memory_usage_psutil():
    # return the memory usage in MB
    import psutil
    process = psutil.Process(os.getpid())
    mem = process.get_memory_info()[0] / float(2 ** 20)
    return mem



def xspaceSamplerFromHvalue(h1step,h2step,h3step):
        start = timer()

        ###############################################################################
        ## X \in \R^XSPACE_DIMENSION, the space of points, 
        ## each constraint to one environment (E) box
        ###############################################################################

        ## h3 is the height of the head, which we fix here to investigate the factors
        ## which define the surface which induces the same linear subspace in X
        ## Let h1 be the height of the hip

        minH1 = 100000
        maxH1 = 0
        minH2 = 100000
        maxH2 = 0
        minH3 = 100000
        maxH3 = 0

        h1low = 0.2
        h1high = 0.8
        h2low = -0.2
        h2high = 0.8
        h3low = 0.7
        h3high = 1.7
        VIDEO_DEBUG = 0

        h3=h3low
        NCtr = 0
        NfeasibleCtr = 0
        XLarray = []
        XRarray = []
        XMarray = []
        Harray = []
        THETAarray=[]
        Qarray=[]

        imgCtr=0

        while h3 <= h3high:
                h3 = h3+h3step

                ## check the size of array (if only one, then continue)
                M = 0

                ##iterate over homotopy classes (HC)
                K_homotopy = 1

                h2 = h2low

                while h2<=h2high:

                        h2=h2+h2step
                        h1 = h1low

                        while h1 <= h1high:

                                NCtr = NCtr+1
                                h1 = h1+h1step
                                [xL,xM,xR,q,theta] = hspace2xspace(K_homotopy,h1,h2,h3)

                                if xL is not None:

                                        if h1 >= maxH1:
                                                maxH1 = h1
                                        if h1 <= minH1:
                                                minH1 = h1
                                        if h2 >= maxH2:
                                                maxH2 = h2
                                        if h2 <= minH2:
                                                minH2 = h2
                                        if h3 >= maxH3:
                                                maxH3 = h3
                                        if h3 <= minH3:
                                                minH3 = h3

                                        NfeasibleCtr = NfeasibleCtr+1
                                        M = M+1

                                        XLarray.append(xL)
                                        XRarray.append(xR)
                                        XMarray.append(xM)
                                        THETAarray.append(theta)
                                        Qarray.append(q)
                                        Harray.append([K_homotopy,h1,h2,h3])

                                        imgCtr=imgCtr+1
                                        if VIDEO_DEBUG:
                                                xspaceToImage(xL,xM,xR,imgCtr)

                                        ### add mirror k=3
                                        qmirror = -q
                                        [xLmirror,xMmirror,xRmirror] = qtox(qmirror)
                                        if q is None:
                                                print "mirror error"
                                                sys.exit(0)
                                        thetamirror = theta

                                        XLarray.append(xLmirror)
                                        XRarray.append(xRmirror)
                                        XMarray.append(xMmirror)
                                        THETAarray.append(thetamirror)
                                        Qarray.append(qmirror)
                                        Harray.append([3,h1,h2,h3])
                                #### display x

                #print "for h1 in",hmin,hmax
                Nsamples=len(XLarray)
                mem = memory_usage_psutil()
                print "h3=",h3," and points=",M," (total:",Nsamples,", memory usage:",mem,")"
                if mem > 14000:
                        print "memory limit reached:",mem
                        sys.exit(1)

        NfeasibleCtrReduced = len(XLarray)

        hfolder=folderNameFromHvalues(h1step,h2step,h3step)

        output_folder = os.environ["MPP_PATH"]+"mpp-robot/output/xspace"+hfolder
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)

        XLname =   output_folder+"/xsamplesL.dat"
        XMname =   output_folder+"/xsamplesM.dat"
        XRname =   output_folder+"/xsamplesR.dat"
        THETAname =output_folder+"/xsamplesTheta.dat"
        Qname =    output_folder+"/xsamplesQ.dat"
        Hname =    output_folder+"/hsamples.dat"
        HeadName = output_folder+"/headersamples.dat"

        pickle.dump( XLarray, open( XLname, "wb" ) )
        pickle.dump( XMarray, open( XMname, "wb" ) )
        pickle.dump( XRarray, open( XRname, "wb" ) )
        pickle.dump( Harray, open( Hname, "wb" ) )
        pickle.dump( Qarray, open( Qname, "wb" ) )
        pickle.dump( THETAarray, open( THETAname, "wb" ) )

        pickle.dump( [XSPACE_DIMENSION, VSTACK_DELTA, heights], open( HeadName, "wb" ) )

        print "======================================================================="
        print "Samples:"
        print "  N   = "+str(NCtr)
        print "  N_f = "+str(NfeasibleCtrReduced)
        print "======================================================================="
        print "interval of H values"
        print "h1=\["+str(minH1)+","+str(maxH1)+"\]"
        print "h2=\["+str(minH2)+","+str(maxH2)+"\]"
        print "h3=\["+str(minH3)+","+str(maxH3)+"\]"
        print "======================================================================="
        end = timer()

        ts= np.around(end - start,2)

        print "================================================================"
        print "Time elapsed for sampling xspace configurations"
        print "================="
        print ts,"s"
        print "================================================================"
        print "data written to folder",output_folder
        print "================================================================"

        if VIDEO_DEBUG and NfeasibleCtrReduced > 0:
                folder = "xspaceWalk"
                rmrfstr = "rm -rf ../data/"+folder+"/out.mp4"
                os.system(rmrfstr)
                ffmpegstr = "ffmpeg -y -framerate 8 -start_number 0 -i ../data/"+folder+"/xspaceWalk%d.png -pix_fmt yuv420p ../data/"+folder+"/out.mp4"
                vlcstr = "vlc ../data/"+folder+"/out.mp4"
                os.system(ffmpegstr)
                os.system(vlcstr)

if __name__ == '__main__':
        xspaceSamplerFromHvalue(SAMPLER_H1_STEP,SAMPLER_H2_STEP,SAMPLER_H3_STEP)
