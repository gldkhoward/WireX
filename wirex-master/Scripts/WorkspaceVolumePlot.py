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
# Name:        workspaceVolumePlot
# Purpose:
#
# Author:      asp
#
# Created:     19.08.2015
#-------------------------------------------------------------------------------
#!/usr/bin/env python

import numpy as np
from mpl_toolkits.mplot3d import axes3d
from scipy.interpolate import griddata
import matplotlib.pyplot as plt
from matplotlib import cm
import csv
import htmlreport
import WiPy

def ComputeWorkspaceVolume():
    Iws.setIterations(3)
    Iws.setMethod(2,0)

    delta = 1.0
    x = np.arange(0, 10.0, delta)
    y = np.arange(0, 10.0, delta)
    # generate the grid based on the settings above
    X, Y = np.meshgrid(x, y)
    Z = np.zeros_like(X)
    print len(x), len(y)
    for i in range(0,len(x)):
        for j in range(0,len(y)):
            alpha = X[i,j]*2.0*3.1415926/360.0
            beta = Y[i,j]*2.0*3.1415926/360.0
#            print i, alpha

            Iws.setOrientation(0,beta,alpha)
            Iws.setProjectionCenter(0,0,1)
            Iws.calculateWorkspace()
            res = Iws.calculateWorkspaceProperties()
            Z[i,j] = res['volume']
            print alpha, beta
            print "Workspace Volume: " + str( res['volume'] )
    #        print "workspace Surface: " + str( res['surface'] )

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot_wireframe(X,Y,Z,cmap=cm.coolwarm)
#    ax.view_init(elev=10., azim=135)
    ax.set_xlabel('alpha [deg]')
    ax.set_ylabel('beta [deg]')
    ax.set_zlabel('volume V [m*m*m]')
    plt.savefig('IPAnema1_COWS_Volume.png')
    plt.savefig('IPAnema1_COWS_Volume.eps')
    plt.close()

def main():
    print "WiPy Version", WiPy.version()
    WiPy.createRobot()
    robotName='IPAnema'
    if not (Irobot.load(robotName+'.wcrfx')):
        print "Loading of robot configuration failed!"
    else:
        print robotName+'.wcrfx'+ " loaded"
    print Irobot.getNow()
    for i in range(0,8):
        print Irobot.getLeg(i)

    if (not Iws.calculateWorkspace()):
        print "Error while workspace computation"
    if not Iws.calculateWorkspaceProperties():
        print "Error while computing propeties"
    print "Volume:", Iws.getWorkspaceVolume()

    ComputeWorkspaceVolume()
    print "Done"
    pass

if __name__ == '__main__':
    main()
