from pylab import *
from timeit import default_timer as timer
import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-environment/mpp")

import random as rnd
from math import cos,sin,tan,pi,asin,acos,atan2,atan
import pickle
import numpy as np
from robot.robotspecifications import *
from robot.htoq import *
from robot.qtox import *

from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection 
import matplotlib.pyplot as plt 
import os 

heights = np.zeros((XSPACE_DIMENSION,1))
heights[0]=0

##DEBUG:
DEBUG=1

for i in range(1,len(heights)):
        heights[i] = VSTACK_DELTA+heights[i-1]

def hspace2xspace(k,h1,h2,h3):

        [q,theta] = htoq(k,h1,h2,h3)

        ###############################################################################
        ### check limits
        ###############################################################################

        ## real limits from the hardware (relative to the previous joint)
        thetaL = THETA_LIMITS_LOWER
        thetaU = THETA_LIMITS_UPPER
        if not((theta<=thetaU).all() and (theta>=thetaL).all()):
                return [None,None,None,None,None]

        ## self collision inspection
        thetaL = THETA_SELFCOLLISION_LOWER
        thetaU = THETA_SELFCOLLISION_UPPER
        if not((theta<=thetaU).all() and (theta>=thetaL).all()):
                return [None,None,None,None,None]

        ###############################################################################
        ## q -> x
        ###############################################################################
        [xL,xM,xR] = qtox(q)
        if xL is None:
                return [None,None,None,None,None]

        ###############################################################################
        return [xL,xM,xR,q,theta]

def xspaceDisplay(xL,xM,xR):
        xspacePlot(xL,xM,xR)
        plt.pause(0.1)

def xspacePlot(xL,xM,xR,show=False):
        fig=figure(1)
        fig.clf()
        ax = fig.gca()

        plt.gca().set_aspect('equal', adjustable='box')
        lenlines=0.6
        for i in range(0,len(heights)):
                plot([-lenlines,lenlines],[heights[i],heights[i]],'-k',linewidth=2)
        
        env=np.zeros((25,2))
        env[ 0 ]=[ -0.0899856966569 , 0.319985696657 ]
        env[ 1 ]=[ -0.0899856950575 , 0.319985696295 ]
        env[ 2 ]=[ -0.0299852151502 , 0.439985215036 ]
        env[ 3 ]=[ 1.40941414287e-05 , 0.449985873245 ]
        env[ 4 ]=[ 1.4103279401e-05 , 0.449985906137 ]
        env[ 5 ]=[ 1.43041187898e-05 , 0.409985696549 ]
        env[ 6 ]=[ -0.119981741564 , 0.379981741604 ]
        env[ 7 ]=[ -0.159985206983 , 0.189985207802 ]
        env[ 8 ]=[ -0.209985515515 , 0.189985515328 ]
        env[ 9 ]=[ -0.239985931476 , 0.189985931674 ]
        env[ 10 ]=[ -0.24998562944 , 0.209985629052 ]
        env[ 11 ]=[ -0.249985906096 , 0.199985903555 ]
        env[ 12 ]=[ -0.249985905854 , 0.19998590603 ]
        env[ 13 ]=[ -0.23998521525 , 0.229985214738 ]
        env[ 14 ]=[ -0.239982886739 , 0.249982886714 ]
        env[ 15 ]=[ -0.23997761043 , 0.289977610651 ]
        env[ 16 ]=[ -0.309191053821 , 0.289991116353 ]
        env[ 17 ]=[ -0.309162274999 , 0.539162274991 ]
        env[ 18 ]=[ -0.309162274994 , 0.539162274992 ]
        env[ 19 ]=[ -0.279957721233 , 0.529957721157 ]
        env[ 20 ]=[ -0.129985629093 , 0.329985629091 ]
        env[ 21 ]=[ -0.129992770764 , 0.209992770792 ]
        env[ 22 ]=[ -0.139975759854 , 0.179975760194 ]
        env[ 23 ]=[ -0.139975759858 , 0.179975760135 ]
        env[ 24 ]=[ -0.149975998286 , 0.159975997964 ]

        env=-env

        for i in range(0,len(env)):
                plot([env[i][0],env[i][1]],[heights[i],heights[i]],'-',color='#CCCCCC',linewidth=2)

        ax.scatter(xL,heights,marker='o',c='r')
        plot(xL,heights,'-r')
        ax.scatter(xR,heights,marker='o',c='r')
        plot(xR,heights,'-r')
        if xM is not None:
                ax.scatter(xM,heights,marker='o',c='r')
                plot(xM,heights,'-r')
        if show:
                plt.show()
def xspaceToImage(xL,xM,xR,did):
        xspacePlot(xL,xM,xR)
        fname = "../data/xspaceWalk/xspaceWalk"+str(did)+".png"
        savefig(fname, bbox_inches='tight')
