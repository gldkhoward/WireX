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
# Name:        wcCsv
# Purpose:     wcCsv supports reading and processing of data files generated
#              with WireCenter.
#
# Author:      asp
#
# Created:     15.08.2015
#-------------------------------------------------------------------------------
#!/usr/bin/env python

import csv
import numpy
from scipy.interpolate import griddata

class wcCsv:
    """
    WcCsv supports reading and processing of data files generated with
    WireCenter's PoseEvaluation Framework.
    """

    def __init__(self):
        """
        Construct an empty instance
        """
        self.srcFilename = ""
        self.x = []  # the column with x-coordinates
        self.xi = [] # 1D array interpolating the range of the x column with resX steps
        self.y = []  # the column with y-coordinates
        self.yi = [] # 1D array interpolating the range of the y column with resY steps
        self.z = []  # the column with z-coordinates
        self.t = []  # the column with t-coordinates
        self.X = []  # 2D array based on xi steps
        self.Y = []  # 2D array based on xi steps
        self.C = []  # 2D array computed from a colum interpolated over (X,Y) array
        self.values = [] # data matrix read from the csv file
        self.col_headers = [] # list with the names (column headers)
        self.csvraw = [] # the raw data extracted from the csv file


    def load(self, filename):
        """
        Load the file into memory and extract the first four columns with
        t,x,y,z
        """
        self.srcFilename = filename+'.csv'
        # read the data table account to the file format written by CPoseEvaluator
        self.values = numpy.genfromtxt(self.srcFilename,delimiter=',',skip_header=1)
        # read the table header from the file
        with open(self.srcFilename, "r") as f:
            self.csvraw = list(csv.reader(f))
        self.col_headers = self.csvraw[0][0:]

        self.t = self.values[:,1]
        self.x = self.values[:,2]
        self.y = self.values[:,3]
        self.z = self.values[:,4]


    def Print(self):
        """
        Print some information about the loaded csv data table.
        """
        print "loaded " + self.srcFilename + " into memory."
        print "The csv table has " + str(len(self.t)) + " entries."
        print "The csv table has " + str(len(self.col_headers)) + " columns."
        i=0
        for s in self.col_headers:
            print i, ":", s
            i+=1


    def interpolateXY(self,resX,resY):
        """
        Prepare the loaded data for plotting. Use the x,y data in the file to
        interpolate the data sets to a regular x,y grid.
        """
        self.xi = numpy.linspace(min(self.x), max(self.x),resX)
        self.yi = numpy.linspace(min(self.y), max(self.y),resY)
        self.X,self.Y = numpy.meshgrid(self.xi,self.yi)


    def interpolateColumnXY(self, col):
        """
        Interpolate the given column (col) on the grid created by interpolateXY()
        Stores the interpolated column in self.C for later use.
        """
        if len(self.xi)==0 or len(self.yi)==0:
            return none
        C = self.values[:,col]

        # automatic resample the data to resX x resY grid for the contour plot
        self.C = griddata((self.x, self.y), C, (self.xi[None,:], self.yi[:,None]), method='cubic')


    def cols(self):
        """ return the number of data columns in the data set """
        return len(self.col_headers)


    def rows(self):
        """ return the number of data rows in the data set """
        return len(self.values)

# The second part of the toolkit. This file containts the diagram templates in
# an own class.

from mpl_toolkits.mplot3d import axes3d
from scipy.interpolate import griddata
import matplotlib.pyplot as plt
from matplotlib import cm

class wcDiagramTemplate:
    """ Dev version. """

    def __init__(self,filename,outputFormat,report):
        self.generateHtml = True
        self.filename = filename
        self.outputFormat = outputFormat
        self.report = []
        self.hasTitle = True
        if (report):
            self.report = report

    def Save(self, fig, colheader, title, desc):
        """ save the plot to disk and possible in the report file """
        if self.hasTitle:
            plt.title(colheader)
        plt.savefig(self.filename+'_'+colheader+'_'+title+self.outputFormat)
        if self.generateHtml:
            self.report.newSubSection(colheader+' '+desc)
            self.report.newFigure(fig)
        plt.close()


    def ScatterPlot3D(self, X,Y,Z):
        """
        Generate a 3D scatter plot using the data in X,Y,Z
        """
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.scatter(X,Y,Z)
        ax.set_xlabel('$x$ [m]')
        ax.set_ylabel('$y$ [m]')
        ax.set_zlabel('$z$ [m]')
        plt.savefig(self.filename+'_Scatter3D'+self.outputFormat)
        if self.generateHtml:
            self.report.newParagraph('Evalution performed on the following grid:')
            self.report.newFigure(fig)
        plt.close()


    def ScatterPlot2D(self, X, Y):
        fig = plt.figure()
        plt.scatter(X,Y)
        plt.xlabel('$x$ [m]')
        plt.ylabel('$y$ [m]')
        plt.legend()
        plt.grid(True)
        plt.savefig(self.filename+'_Scatter3D'+self.outputFormat)
        if self.generateHtml:
            self.report.newParagraph('Evalution performed on the following grid:')
            self.report.newFigure(fig)
        plt.close()


    def ContourPlot(self, xi,yi,C, colheader):
        fig = plt.figure()
        # do the plotting and save the result
        CS = plt.contour(xi, yi, C)

        # add labels to the plot
        plt.clabel(CS, inline=1, fontsize=10)
        plt.xlabel('$x$ [m]')
        plt.ylabel('$y$ [m]')
        plt.legend()
        plt.grid(True)
        self.Save(fig, colheader, 'Contour','Contour Plot')


    def HistogramPlot(self, values, colheader):
        fig = plt.figure()
        next, bins, patches = plt.hist(values,50,normed=1, histtype='bar', rwidth=0.8, log=True) #) # use for logarithmic scale in the histo
        plt.setp(patches,'facecolor', 'b')#, 'alpha', 0.75)
        plt.grid(True)

        self.Save(fig, colheader, 'Histogram','Histogram Plot')


    def Scatter3DPlot(self, x, y, z, colheader):
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.scatter(x,y,z)
        ax.set_xlabel('$x$ [m]')
        ax.set_ylabel('$y$ [m]')
        self.Save(fig,colheader,'Scatter','Scatter Plot')


    def Surface3DPlot(self,X,Y,C,colheader):
        # create a surface plot
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.plot_surface(X,Y,C,cmap=cm.coolwarm)
        ax.set_xlabel('$x$ [m]')
        ax.set_ylabel('$y$ [m]')
        self.Save(fig,colheader,'Surface','3D Surface Plot')


    def Wireframe3DPlot(self,X,Y,C,colheader):
        # create a wireframe plot
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.plot_wireframe(X,Y,C)
        ax.set_xlabel('$x$ [m]')
        ax.set_ylabel('$y$ [m]')
        self.Save(fig,colheader,'Wireframe','3D Wireframe Plot')


# only for testing purpose
def main():
    dat = wcCsv()
    dat.load("output")
    dat.Print()
    dat.interpolateXY(200,200)
    pass

if __name__ == '__main__':
    main()
