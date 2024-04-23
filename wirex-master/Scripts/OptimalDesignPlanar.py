# 
# WireX
#
# Copyright (c) 2006-2019 Andreas Pott
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# 
#-------------------------------------------------------------------------------
# Name:        OptimalDesignPlanar
# Purpose:     Analyzed the optimal design of a simple planar CDPR with respect
#              to maximizing the size of the workspace.
#
# Author:      asp
#
# Created:     02.09.2015
#-------------------------------------------------------------------------------
#!/usr/bin/env python

import WiPy
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import numpy as np
from scipy.optimize import minimize
import scipy

def AlgorihtmSettings():
    # compute the workspace cross section
    Iws.setWorkspaceCriterion(0)    # force feasiblity
    Iws.setMethod(2,1)
    Iws.setProjectionCenter(0,0,0)
    Iws.setOrientation(0,0,0)
    Iws.setIterations(5)
    Iws.setEps(0.000001)
    Iws.setOrientationRequirement(0)    # 0 relates to inclusion; 1 to total orienation workspace
    Iws.synchronizeSettings()       # we have to propagete the setting to the cross section

def generateRobot():
    """
    Generate an robot based on little parameters
    """
    WiPy.createRobot()
    Irobot.setMotionPattern(4,3)
    Irobot.setBase(0, -1.2, 1, 0)
    Irobot.setBase(1,  1.2, 1, 0)
    Irobot.setBase(2,  1.5,-1, 0)
    Irobot.setBase(3, -1.5,-1, 0)

    Irobot.setPlatform(0,-0.05, 0.1, 0)
    Irobot.setPlatform(1, 0.05, 0.1, 0)
    Irobot.setPlatform(2, 0.05,-0.1, 0)
    Irobot.setPlatform(3,-0.05,-0.1, 0)

def setGeometry(length_base,length_platform=0.1, vol_platform=.02):
    vol_base =1.
    width_base = vol_base/length_base
    width_platform = vol_platform /length_platform

    Irobot.setBase(0, -length_base/2.,  width_base/2., 0)
    Irobot.setBase(1,  length_base/2.,  width_base/2., 0)
    Irobot.setBase(2,  length_base/2., -width_base/2., 0)
    Irobot.setBase(3, -length_base/2., -width_base/2., 0)

    Irobot.setPlatform(0, -length_platform/2.,  width_platform/2., 0)
    Irobot.setPlatform(1,  length_platform/2.,  width_platform/2., 0)
    Irobot.setPlatform(2,  length_platform/2., -width_platform/2., 0)
    Irobot.setPlatform(3, -length_platform/2., -width_platform/2., 0)


def computeWorkspace():
    """
    compute the 2d workspace border and return the volumen of the workspace
    """
    # calculate the workspace and get the data
    Iws.setProjectionCenter(0,0,0)
    Iws.calculateWorkspaceCrosssection('z')
    CS = Iws.getWorkspaceCrosssectionMatrix()
    # we only get the vertices but not the area :-(
    vol = 0
    for v in CS:
        vol += (v[0]*v[4]-v[3]*v[1])**2
    vol /= 2.

    return vol

def parameterSweep():
    generateRobot()
    AlgorihtmSettings()
    LB = np.arange(0.21, 5.0, 0.2)
    LP = np.arange(0.01,0.2,0.01)
    X,Y = np.meshgrid(LB, LP)
    print X.shape
    V = np.zeros_like(X)
    print V.shape

    print len(LB)
    print len(LP)

    for j in range(0,len(LB)-1):
        for i in range(0,len(LP)-1):
            length_b = LB[j]
            length_p = LP[i]
            setGeometry(length_b,length_p)
            V[i,j] = computeWorkspace()
            print i,",",j," l: ",length_b,"  vol: ", V[i,j]

#    print LB,V
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot_wireframe(X,Y,V)
#    plt.grid('on')
    plt.show()
    
def objVolumeAlpha(x):
    setGeometry(x)
    return computeWorkspace()
    
def objVolumeAlphaBeta(x):
    print x
    setGeometry(x[0],x[1])
    return -computeWorkspace()
    
path = []
def objVolumeAlphaBetaVol(x):
    print x
    path.append(x)
    setGeometry(x[0],x[1],x[2]**2)
    return -computeWorkspace()

def addRobotToPlot(fig):
    """
    Helper function to add some geometric items sketching the robot geometry
    The geometry parameters of the robot are extracted through WiPy from the current
    robot session. When using with preprocesses data, make sure you loaded the
    respective robot gemetriy first.
    """
    # enhance size to show the labels
    plt.xlim([-1.8,1.8])
    plt.ylim([-1.3,1.3])
    # draw robot hull
    platformpoly = []
    basepoly = []

    for i in range(0,Irobot.getNow()):
        ai = Irobot.getBase(i)
        basepoly.append((ai[0],ai[1]))
        bi = Irobot.getPlatform(i)
        platformpoly.append((bi[0],bi[1]))

    poly = plt.Polygon(platformpoly,fill=False)
    fig.gca().add_artist(poly)
    poly = plt.Polygon(basepoly,fill=False)
    fig.gca().add_artist(poly)

    for i in range(0,Irobot.getNow()):
        ai = Irobot.getBase(i)
        circle1 = plt.Circle((ai[0],ai[1]),0.02)
        fig.gca().add_artist(circle1)
        txt = plt.Text(ai[0],ai[1],'$A$')
        fig.gca().add_artist(txt)

        bi = Irobot.getPlatform(i)
        circle1 = plt.Circle((bi[0],bi[1]),0.02)
        fig.gca().add_artist(circle1)
        txt = plt.Text(bi[0],bi[1],'$B$')
        fig.gca().add_artist(txt)

        line = plt.Line2D([ai[0],bi[0]],[ai[1],bi[1]],linewidth=0.5)
        fig.gca().add_artist(line)


def printRobotGeometry():
    """
    extract and print the geometry of the robot using the Irobot interface
    """
    print "WiPy Version", WiPy.version()
    print "Robot Now: ", Irobot.getNow()

    for i in range(0,Irobot.getNow()):
        print "Leg", i+1,": [a,b]_i =", Irobot.getBase(i),",", Irobot.getPlatform(i)

        
def rosen(x):
    """The Rosenbrock function"""
    return sum(100.0*(x[1:]-x[:-1]**2.0)**2.0 + (1-x[:-1])**2.0)

def main():
    generateRobot()
    AlgorihtmSettings()

#    x0 = np.array([1.3, 0.7, 0.8, 1.9, 1.2])
#    print scipy.fmin(rosen,x0)
#    res = minimize(rosen, x0, method='nelder-mead', options={'xtol': 1e-8, 'disp': True})
#    print res
    print '\n'
    x0 = x0 = np.array([.99, 0.1, 0.11])
#    print minimize(objVolumeAlphaBeta,x0,method='nelder-mead', options={'xtol': 1e-4, 'disp': True})
    print minimize(objVolumeAlphaBetaVol,x0,method='Nelder-Mead', options={'xtol': 1e-3, 'disp': True})

    fig = plt.figure()
    addRobotToPlot(fig)
    printRobotGeometry()
    plt.show()

#    parameterSweep()
    pass

if __name__ == '__main__':
    main()



