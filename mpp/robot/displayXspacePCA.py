import sys,os
sys.path.append(os.environ["MPP_PATH"]+"mpp-robot/mpp")

import numpy as np
from pylab import *
from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
from sklearn import preprocessing
from robot.plot_trisurf_fixed import *

def getMainPCAaxes(Xarray):
        N = len(Xarray[0])
        if len(Xarray)>0:
                ## Xarray to MxN numpy array
                xx = np.zeros((N,len(Xarray)))
                for i in range(0,len(Xarray)):
                        for j in range(0,N):
                                xx[j,i] = Xarray[i][j]

                xx = preprocessing.scale(xx)
                [U,S,V]=np.linalg.svd(xx)
                #print np.around(S,1)
                uu = np.around(U,2)

                X1 = uu[:,0]
                X2 = uu[:,1]
                X3 = uu[:,2]

                return [X1,X2,X3,np.around(S,1)]

def projectDataOnto3MainAxes(Xarray):
                Npts = len(Xarray[0])
                PCaxes2 = getMainPCAaxes(Xarray)
                [X1,X2,X3,S] = PCaxes2

                ## Xarray to MxN numpy array
                xx = np.zeros((Npts,len(Xarray)))
                for i in range(0,len(Xarray)):
                        for j in range(0,Npts):
                                xx[j,i] = Xarray[i][j]

                xx = preprocessing.scale(xx)

                Xproj = np.zeros((3,len(Xarray)))
                for i in range(0,len(Xarray)):
                        x = np.dot(X1.T,xx[:,i])
                        y = np.dot(X2.T,xx[:,i])
                        z = np.dot(X3.T,xx[:,i])
                        Xproj[0,i] = x
                        Xproj[1,i] = y
                        Xproj[2,i] = z

                X = Xproj[0,:]
                Y = Xproj[1,:]
                Z = Xproj[2,:]
                return [X,Y,Z]

### backward compability
def plotDataMainAxes(Xarray,PCaxes,h3,Npts,fname):
        plotDataMainAxes(Xarray,fname, height=h3)

def plotDataMainAxes(Xarray,fname, height=None):
        if not len(Xarray)>0:
                return False
        Npts = len(Xarray[0])
        if len(Xarray)>Npts:
                PCaxes2 = getMainPCAaxes(Xarray)
                [X1,X2,X3,S] = PCaxes2

                ## Xarray to MxN numpy array
                xx = np.zeros((Npts,len(Xarray)))
                for i in range(0,len(Xarray)):
                        for j in range(0,Npts):
                                xx[j,i] = Xarray[i][j]

                xx = preprocessing.scale(xx)

                Xproj = np.zeros((3,len(Xarray)))
                for i in range(0,len(Xarray)):
                        x = np.dot(X1.T,xx[:,i])
                        y = np.dot(X2.T,xx[:,i])
                        z = np.dot(X3.T,xx[:,i])
                        Xproj[0,i] = x
                        Xproj[1,i] = y
                        Xproj[2,i] = z

                X = Xproj[0,:]
                Y = Xproj[1,:]
                Z = Xproj[2,:]

                fig=figure(1)
                fig.clf()
                ax = fig.gca(projection='3d')
                ax.scatter(X,Y,Z,marker='o',c='r',s=5)

                #ax.set_xlim3d(-8, 8)
                #ax.set_ylim3d(-6, 6)
                #ax.set_zlim3d(-6, 6)

                variability = np.sum(S[0:3])/float(np.sum(S[0:]))
                print np.around(S,2)
                print "data variability on 3 main axes:",variability

                if height is not None:
                        ax.text(-7,-6,3, "height="+str(np.around(float(height),3)), None)
                ax.text(-7,-8,3, "variability="+str(np.around(variability,2)), None)
                savefig( fname, dpi=gcf().dpi)
                return True
        else:
                return False


def plotDataMainAxesSurface(Xarray,fname, height=None):
        if not len(Xarray)>0:
                return False

        Npts = len(Xarray[0])
        if len(Xarray)>Npts:
                PCaxes2 = getMainPCAaxes(Xarray)
                [X1,X2,X3,S] = PCaxes2


                ## Xarray to MxN numpy array
                xx = np.zeros((Npts,len(Xarray)))
                for i in range(0,len(Xarray)):
                        for j in range(0,Npts):
                                xx[j,i] = Xarray[i][j]

                xx = preprocessing.scale(xx)

                Xproj = np.zeros((3,len(Xarray)))
                for i in range(0,len(Xarray)):
                        x = np.dot(X1.T,xx[:,i])
                        y = np.dot(X2.T,xx[:,i])
                        z = np.dot(X3.T,xx[:,i])
                        Xproj[0,i] = x
                        Xproj[1,i] = y
                        Xproj[2,i] = z

                X = Xproj[0,:]
                Y = Xproj[1,:]
                Z = Xproj[2,:]

                fig=figure(1)
                fig.clf()
                ax = fig.gca(projection='3d')
                #ax.scatter(X,Y,Z,marker='o',c='r',s=5)
                plot_trisurf_fixed(ax,X,Y,Z,cmap=cm.jet, linewidth=0,vmax=0.1)

                #ax.set_xlim3d(-8, 8)
                #ax.set_ylim3d(-6, 6)
                #ax.set_zlim3d(-6, 6)

                variability = np.sum(S[0:3])/float(np.sum(S[0:]))
                print np.around(S,2)
                print "data variability on 3 main axes:",variability

                return True
        else:
                return False
