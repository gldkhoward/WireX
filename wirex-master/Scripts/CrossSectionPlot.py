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
# -------------------------------------------------------------------------------
# Name:        Generate Cross Section Plots
#              Diagram generator for figures in the WireBook project
# Purpose:
#
# Author:      asp
#
# Created:     09.08.2015
#-------------------------------------------------------------------------------
#!/usr/bin/env python

import WiPy
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

def generateRobot():
    WiPy.createRobot()
	# geometry definition
    Irobot.setMotionPattern(4,3)
    Irobot.setModelParameter("PlanarRobot","frameLength",3.0)
    Irobot.setModelParameter("PlanarRobot","frameWidth",2.0)
    Irobot.setModelParameter("PlanarRobot","platformLength",0.1)
    Irobot.setModelParameter("PlanarRobot","platformWidth",0.2)
    Irobot.applyModel("PlanarRobot")

def generateRobot2():
    """
    Generate an asymmetric robot
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

    ttopt = [dict(horizontalalignment='right',verticalalignment='bottom'),
             dict(horizontalalignment='left',verticalalignment='bottom'),
             dict(horizontalalignment='left',verticalalignment='top'),
             dict(horizontalalignment='right',verticalalignment='top')]
    for i in range(0,Irobot.getNow()):
        ai = Irobot.getBase(i)
        circle1 = plt.Circle((ai[0],ai[1]),0.02)
        fig.gca().add_artist(circle1)
        txt = plt.Text(ai[0],ai[1],'$A_'+str(i+1)+'$',**ttopt[i])
        fig.gca().add_artist(txt)

        bi = Irobot.getPlatform(i)
        circle1 = plt.Circle((bi[0],bi[1]),0.02)
        fig.gca().add_artist(circle1)
        txt = plt.Text(bi[0],bi[1],'$B_'+str(i+1)+'$',**ttopt[i])
        fig.gca().add_artist(txt)

        line = plt.Line2D([ai[0],bi[0]],[ai[1],bi[1]],linewidth=0.5)
        fig.gca().add_artist(line)


def pltCrossSection(CS):
    # we have the required data; make the plots
    # transform the point data of the cross section into the data model used
    # for matplotlib
    CSX = []
    CSY = []

    for v in CS:
        CSX.extend([v[0],v[3]])
        CSY.extend([v[1],v[4]])

    plt.plot(CSX, CSY, color='black', linewidth=2)


def printRobotGeometry():
    """
    extract and print the geometry of the robot using the Irobot interface
    """
    print "WiPy Version", WiPy.version()
    print "Robot Now: ", Irobot.getNow()

    for i in range(0,Irobot.getNow()):
        print "Leg", i+1,": [a,b]_i =", Irobot.getBase(i),",", Irobot.getPlatform(i)


def AlgorihtmSettings():
    # compute the workspace cross section
    Iws.setWorkspaceCriterion(0)    # force feasiblity
    Iws.setMethod(2,1)
    Iws.setProjectionCenter(0,0,0)
    Iws.setOrientation(0,0,0)
    Iws.setIterations(5)
    Iws.setEps(0.001)


def generateInclusionOrientationWorkspace():
    """
    compute and plot the inclusion orienation workspace
    """
    fig = plt.figure()

    # compute the workspace cross section for a given orientation
    Iws.createOrientationSet(0,0,3.0*3.1415/180.0,30)
    Iws.setOrientationRequirement(0)    # 0 relates to inclusion; 1 to total orienation workspace
    Iws.synchronizeSettings()       # we have to propagete the setting to the cross section
    # calculate the workspace and get the data
    Iws.calculateWorkspaceCrosssection('z')
    CS = Iws.getWorkspaceCrosssectionMatrix()
    pltCrossSection(CS)

    # the following code deals with formating and saving the plot
    matplotlib.rcParams['xtick.direction'] = 'out'
    matplotlib.rcParams['ytick.direction'] = 'out'

    plt.xlabel('$x$ [m]')
    plt.ylabel('$y$ [m]')
    plt.xlim([-1.5,1.5])
    plt.ylim([-1.0,1.0])
#    plt.title('workspace of the 1R2T robot')
    plt.grid('on')
    plt.savefig('Workspace_1R2T_Inclusion.png', bbox_inches='tight')
    plt.savefig('Workspace_1R2T_Inclusion.eps', bbox_inches='tight')
    # at the end we explicitly clear up
    plt.clf()


def generateMaximumOrientationWorkspace():
    """
    compute and plot the maximum orienation workspace
    """
    fig = plt.figure()

    # compute the workspace cross section for a given orientation
    Iws.createOrientationSet(0,0,90.0*3.1415/180.0,180)
    Iws.setOrientationRequirement(0)    # 0 relates to inclusion; 1 to total orienation workspace
    Iws.synchronizeSettings()       # we have to propagete the setting to the cross section
    # calculate the workspace and get the data
    Iws.calculateWorkspaceCrosssection('z')
    CS = Iws.getWorkspaceCrosssectionMatrix()
    pltCrossSection(CS)

    # the following code deals with formating and saving the plot
    matplotlib.rcParams['xtick.direction'] = 'out'
    matplotlib.rcParams['ytick.direction'] = 'out'

    plt.xlabel('$x$ [m]')
    plt.ylabel('$y$ [m]')
    plt.xlim([-1.5,1.5])
    plt.ylim([-1.0,1.0])
#    plt.title('workspace of the 1R2T robot')
    plt.grid('on')
    plt.savefig('Workspace_1R2T_Maximum.png', bbox_inches='tight')
    plt.savefig('Workspace_1R2T_Maximum.eps', bbox_inches='tight')
    # at the end we explicitly clear up
    plt.clf()


def generateTotalOrientationWorkspace():
    """
    compute and plot the total orientation workspace
    """
    fig = plt.figure()

    # compute the workspace cross section for a given orientation
    Iws.createOrientationSet(0,0,3.0*3.1415/180.0,50)
    Iws.setOrientationRequirement(1)    # 0 relates to inclusion; 1 to total orienation workspace
    Iws.synchronizeSettings()       # we have to propagete the setting to the cross section
    # calculate the workspace and get the data
    Iws.calculateWorkspaceCrosssection('z')
    CS = Iws.getWorkspaceCrosssectionMatrix()

    pltCrossSection(CS)

    # the following code deals with formating and saving the plot
    matplotlib.rcParams['xtick.direction'] = 'out'
    matplotlib.rcParams['ytick.direction'] = 'out'

    plt.xlabel('$x$ [m]')
    plt.ylabel('$y$ [m]')
    plt.xlim([-1.5,1.5])
    plt.ylim([-1.0,1.0])
#    plt.title('workspace of the 1R2T robot')
    addRobotToPlot(fig)
    plt.grid('on')
    plt.savefig('Workspace_1R2T_TotalOrientation.png', bbox_inches='tight')
    plt.savefig('Workspace_1R2T_TotalOrientation.eps', bbox_inches='tight')
    # at the end we explicitly clear up
    plt.clf()


def generateTranslationalWorkspace():
    """
    compute and plot the translational workspace
    """
    fig = plt.figure()

    # compute the workspace cross section for a given orientation
    Iws.setOrientation(0,0,0)#3.0*3.1415/180.0)
    Iws.synchronizeSettings()       # we have to propagete the setting to the cross section
    # calculate the workspace and get the data
    Iws.calculateWorkspaceCrosssection('z')
    CS = Iws.getWorkspaceCrosssectionMatrix()
    pltCrossSection(CS)

    # the following code deals with formating and saving the plot
    matplotlib.rcParams['xtick.direction'] = 'out'
    matplotlib.rcParams['ytick.direction'] = 'out'

    plt.xlabel('$x$ [m]')
    plt.ylabel('$y$ [m]')
    plt.xlim([-1.5,1.5])
    plt.ylim([-1.0,1.0])
    addRobotToPlot(fig)
#    plt.title('workspace of the 1R2T robot')
    plt.grid('on')

    plt.savefig('Workspace_1R2T_Translational.png', bbox_inches='tight')
    plt.savefig('Workspace_1R2T_Translational.eps', bbox_inches='tight')
    # at the end we explicitly clear up
    plt.clf()


def generateFullWorkspace():
    """
    plot multiple cross sections in only one 2D plot
    """
    fig = plt.figure()
#    ax = fig.add_subplot(111, projection='3d')
    for i in range(-5,6):
        # compute the workspace cross section for a given orientation
        phi =i*3.1415/180.0
        Iws.setOrientation(0,0,phi)
        Iws.synchronizeSettings()       # we have to propagete the setting to the cross section
        # calculate the workspace and get the data
        Iws.calculateWorkspaceCrosssection('z')
        CS = Iws.getWorkspaceCrosssectionMatrix()

        # we have the required data; make the plots
        # transform the point data of the cross section into the data model used
        # for matplotlib
        CSX = []
        CSY = []

        for v in CS:
            CSX.extend([v[0],v[3]])
            CSY.extend([v[1],v[4]])

        plt.plot(CSX,CSY)

    # plt.plot(CSX, CSY, color='black', linewidth=2)

    # the following code deals with formating and saving the plot
    matplotlib.rcParams['xtick.direction'] = 'out'
    matplotlib.rcParams['ytick.direction'] = 'out'

    plt.xlabel('$x$ [m]')
    plt.ylabel('$y$ [m]')
    plt.xlim([-1.5,1.5])
    plt.ylim([-1.0,1.0])
#    plt.title('workspace of the 1R2T robot')

    plt.grid('on')
    plt.savefig('Workspace_1R2T_GeneralWorkspace2D.png', bbox_inches='tight') # try to compute a smaller bounding box
    plt.savefig('Workspace_1R2T_GeneralWorkspace2D.eps', bbox_inches='tight')
    # at the end we explicitly clear up
    plt.clf()


def generateFullWorkspace3d():
    """
    we want layered cross section of the workspace in one 3D plot
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for i in range(-45,45):
        # compute the workspace cross section for a given orientation
        phi =i*3.1415/180.0
        Iws.setOrientation(0,0,phi)
        Iws.synchronizeSettings()       # we have to propagete the setting to the cross section
        # calculate the workspace and get the data
        Iws.calculateWorkspaceCrosssection('z')
        CS = Iws.getWorkspaceCrosssectionMatrix()

        # we have the required data; make the plots
        # transform the point data of the cross section into the data model used
        # for matplotlib
        CSX = []
        CSY = []
        CSZ = []

        for v in CS:
            CSX.extend([v[0],v[3]])
            CSY.extend([v[1],v[4]])
            CSZ.extend([i,i])

        ax.plot(CSX,CSY,i)

    # plt.plot(CSX, CSY, color='black', linewidth=2)

    # the following code deals with formating and saving the plot
    matplotlib.rcParams['xtick.direction'] = 'out'
    matplotlib.rcParams['ytick.direction'] = 'out'

    plt.xlabel('$x$ [m]')
    plt.ylabel('$y$ [m]')
#    plt.title('workspace of the 1R2T robot')
#    plt.zlabel('$\phi$ [rad]')
#    plt.grid('on')
    plt.savefig('Workspace_1R2T_GeneralWorkspace3D.png', bbox_inches='tight')
    plt.savefig('Workspace_1R2T_GeneralWorkspace3D.eps', bbox_inches='tight')
    # at the end we explicitly clear up
    plt.clf()


def main():
    matplotlib.rcParams.update({'font.size': 16}) # font size 16 for 2D plots, 12 for 3D plots
    generateRobot2()
    printRobotGeometry()
    AlgorihtmSettings()
#    generateFullWorkspace()
#    generateFullWorkspace3d()
    generateTranslationalWorkspace()
#    generateInclusionOrientationWorkspace()
#    generateMaximumOrientationWorkspace()
    generateTotalOrientationWorkspace()


if __name__ == '__main__':
    main()
