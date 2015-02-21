import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")
sys.path.append(os.environ["MPP_PATH"]+"mpp-mathtools/mpp")
from robot.folderNameFromHvalues import *
from robot.robotspecifications import *
from robot.xspaceSamplerFromHvalue import *
from robot.xsamplesToPolytopes import *
from mathtools.timer import Timer

print "Hvalue Step Size:",SAMPLER_H1_STEP,SAMPLER_H2_STEP,SAMPLER_H3_STEP

tt = Timer("building convex sets from configuration cross sections")
xspaceSamplerFromHvalue(SAMPLER_H1_STEP,SAMPLER_H2_STEP,SAMPLER_H3_STEP)
xsamplesToPolytopes()
tt.stop()
