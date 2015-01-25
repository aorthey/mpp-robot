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

from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection 
import matplotlib.pyplot as plt 
import os 

heights = np.zeros((XSPACE_DIMENSION,1))
heights[0]=0

for i in range(1,len(heights)):
        heights[i] = VSTACK_DELTA+heights[i-1]

def qtox(q):
        ## artificial limits imposed by tangens (-pi/2,pi/2)
        tlimit = pi/2
        qaL = np.array((-tlimit,-tlimit,-tlimit,-tlimit,-tlimit))
        qaU = np.array((tlimit,tlimit,tlimit,tlimit,tlimit))
        if not((q<=qaU).all() and (q>=qaL).all()):
                if DEBUG_QTOX:
                        print "q limits not fulfilled",q
                return [None,None,None]

        q1L = QLIMIT_HOMOTOPY01_LOWER
        q1U = QLIMIT_HOMOTOPY01_UPPER
        q2L = QLIMIT_HOMOTOPY23_LOWER
        q2U = QLIMIT_HOMOTOPY23_UPPER

        ##Q has to lie on one of two homotopy class constraints
        if not((q<=q1U).all() and (q>=q1L).all()) \
                and \
           not((q<=q2U).all() and (q>=q2L).all()):
                if DEBUG_QTOX:
                        print "q homotopy constraints not fulfilled",q
                return [None,None,None]


        dk=ROBOT_DIST_FOOT_SOLE
        d0=ROBOT_DIST_KNEE_FOOT
        d1=ROBOT_DIST_HIP_KNEE
        d2=ROBOT_DIST_WAIST_HIP 
        d3=ROBOT_DIST_NECK_WAIST
        d4=ROBOT_DIST_HEAD_NECK

        dtankle = ROBOT_THICKNESS_FOOT_SOLE
        dt0 = ROBOT_THICKNESS_KNEE_FOOT
        dt1 = ROBOT_THICKNESS_HIP_KNEE
        dt2 = ROBOT_THICKNESS_WAIST_HIP
        dt3 = ROBOT_THICKNESS_NECK_WAIST
        dt4 = ROBOT_THICKNESS_HEAD_NECK

        ##update thickness according to curvature
        dtankle = dtankle*0.5
        dt0 = 0.5*dt0/sin(pi/2-q[0])
        dt1 = 0.5*dt1/sin(pi/2-q[1])
        dt2 = dt2*0.5
        dt3 = 0.5*dt3/sin(pi/2-q[3])
        dt4 = 0.5*dt4/sin(pi/2-q[4])

        ###############################################################################
        ## X \in \R^XSPACE_DIMENSION
        ###############################################################################

        ## given h1, compute the 
        ## a : distance from knee to the main axis through foot, hip and waist.
        ## b : distance from neck to the main axis through foot, hip and waist.
        ## http://mathworld.wolfram.com/Circle-CircleIntersection.html

        ## compute q -> x
        xL = np.zeros((XSPACE_DIMENSION,1))
        xM = np.zeros((XSPACE_DIMENSION,1))
        xR = np.zeros((XSPACE_DIMENSION,1))

        knee_height = d0*cos(q[0])+dk
        hip_height = knee_height+d1*cos(q[1])
        waist_height = hip_height+d2*cos(q[2])
        neck_height = waist_height+d3*cos(q[3])
        head_height = neck_height+d4*cos(q[4])

        xctr=1

        xL[0]=-dtankle
        xR[0]=dtankle
        xM[0]=0

        t0 = -tan((q[0]))
        t1 = -tan((q[1]))
        t2 = tan((q[2]))
        t3 = -tan((q[3]))
        t4 = -tan((q[4]))

        ###############################################################################
        ### foot-to-knee path
        ###############################################################################
        while heights[xctr] <= dk:
                xL[xctr] = xL[0]
                xR[xctr] = xR[0]
                xM[xctr] = xM[0]
                xctr=xctr+1

        while heights[xctr] <= knee_height:
                x = (heights[xctr]-dk)*t0
                xL[xctr] = x - dt0
                xR[xctr] = x + dt0
                xM[xctr]=x
                xctr=xctr+1

        ################################################################################
        #### knee-to-hip path
        ################################################################################
        offset = (knee_height-dk)*t0
        kneepos = offset
        while heights[xctr] < hip_height:
                x = (heights[xctr]-knee_height)*t1+offset
                xL[xctr] = x - dt1
                xR[xctr] = x + dt1
                xM[xctr]=x
                xctr=xctr+1

        ################################################################################
        #### hip-to-waist path
        ################################################################################

        offset = (knee_height-dk)*t0+(hip_height-knee_height)*t1
        hippos = offset

        while heights[xctr] < waist_height:
                x = (heights[xctr]-hip_height)*t2+offset
                xL[xctr] = x - dt2
                xR[xctr] = x + dt2
                xM[xctr]=x
                xctr=xctr+1

        ################################################################################
        #### waist-to-neck path
        ################################################################################
        offset = (knee_height-dk)*t0\
                        +(hip_height-knee_height)*t1\
                        +(waist_height-hip_height)*t2

        waistpos = offset

        while heights[xctr] < neck_height:
                x = (heights[xctr]-waist_height)*t3+offset
                xL[xctr] = x - dt3
                xR[xctr] = x + dt3
                xM[xctr]=x
                xctr=xctr+1
        ################################################################################
        #### neck-to-head path
        ################################################################################
        offset = (knee_height-dk)*t0\
                        +(hip_height-knee_height)*t1\
                        +(waist_height-hip_height)*t2\
                        +(neck_height-waist_height)*t3
        neckpos = offset


        while xctr<len(heights) and heights[xctr] < head_height:
                x = (heights[xctr]-neck_height)*t4+offset
                xL[xctr] = x - dt4
                xR[xctr] = x + dt4
                xM[xctr]=x
                xctr=xctr+1

        headpos = (knee_height-dk)*t0\
                        +(hip_height-knee_height)*t1\
                        +(waist_height-hip_height)*t2\
                        +(neck_height-waist_height)*t3\
                        +(head_height-neck_height)*t4

        return [xL,xM,xR]

