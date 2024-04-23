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
# Name:        PE_Preprocessor
# Purpose:     Generate cross section diagrams from result data produced with
#              WireCenter's grid/pose evaluation functions. Furthermore, a
#              histogram is generated for each column in the data file.
#
# Author:      asp/321
#
# Created:     17.05.2015
#-------------------------------------------------------------------------------
#!/usr/bin/env python

# preprocess data written with WireCenter's CPoseEvaluator framework
# for plotting with numby / matplotlib
# currently, the script cannot be run from inside WireCenter due to issues
# when importing numpy and matplotlib inside a WireCenter session
# however, the script can be run from the shell.

from HtmlReport import HtmlReport
from wcCsv import wcCsv
from wcCsv import wcDiagramTemplate

class CrossSectionDiagrams:
    """
    This class support generation of diagrams. The implementation is mostly
    monolithic. Using this class is basically a two step procedure. Make an
    instance and call the main function. However, between these two lines one
    can change the default behaviour simple by overwriting the default settings
    in the class members.
    """
    # configuration
    datafile = 'output'
    srcFilename = datafile+'.csv'

    # select plot types
    doOverviewScatter = True
    doOverview2D = True
    doContour = True
    doHistogram = True
    do3dScatter = True
    do3dSurface = True
    do3dWireframe = True

    # choose settings for the plotting
    resX = 200
    resY = 200
    outputFormat = '.eps'   # choose the file format of the plots through its extension
    generateHtml = True
    hasTitle = True

    # select the columns to plot
    colSelector = [] # range(14,len(col_headers)-1): #

    def __init__(self, filename=None):
        """
        Init the object and assign the given or a default filename for the input
        file.
        """
        if filename is None:
            self.datafile = 'output'
        else:
            self.datafile = filename
        self.srcFilename = self.datafile+'.csv'

        return

    def CreateDiagrams(self):
        """
        The main part of the diagram generator. Read the data table using the csv
        class, loop through the columns and generate the respective diagrams using
        the wcDiagramTemplate class.
        Optionally, the diagrams are also coded into a single HTML report file.
        """
        # generate a html report
        report = HtmlReport()
        report.newReport('report.html')
        report.newSection('Overview')
        DT = wcDiagramTemplate(self.datafile,self.outputFormat,report)
        DT.hasTitle = self.hasTitle

        # load the data from the csv file using the WcCsv engine
        data = wcCsv()
        data.load(self.datafile)
        data.interpolateXY(self.resX,self.resY)

        # write a summary of the settings
        if self.generateHtml:
            report.newParagraph('Number of data points in csv table: '+str(data.rows()))
            report.newParagraph('Number of columns in the csv table: '+str(data.cols()))
            report.newParagraph('Creating plots for the following columns<br>')
            for i in self.colSelector:
                report.write('<li>'+data.col_headers[i]+' ('+str(i)+')</li>')
            report.write('<br>')
            report.write('The following configuration information was stored with the data file:')
            report.newCodeParagraph(self.datafile+'.csv.txt')
            report.newParagraph('The source data file containted the follwing statistics information about the content:')
            report.statisticsOverview(self.datafile+'.csv.stats.html')

        # generate overview plots (not depending on the data columns but on the
        # grid data
        if self.doOverviewScatter:
            DT.ScatterPlot3D(data.x,data.y,data.z)

        if self.doOverview2D:
            DT.ScatterPlot2D(data.x,data.y)

        if self.colSelector==[]:
            self.colSelector = range(14,len(data.col_headers)-1)
        for i in self.colSelector:
            report.newSection(data.col_headers[i])
            # extract the i-th column
            data.interpolateColumnXY(i)
            print "Generating diagrams for column ", i, ": ", data.col_headers[i]

            if self.doContour:
                DT.ContourPlot(data.xi, data.yi, data.C, data.col_headers[i])

            # now we add a history of the data column
            if self.doHistogram:
                DT.HistogramPlot(data.values[:,i], data.col_headers[i])

            # add a 3d plot showing the values over the cross section as a scatter plot
            if self.do3dScatter:
                DT.Scatter3DPlot(data.x,data.y,data.values[:,i],data.col_headers[i])

            # create a surface plot
            if self.do3dSurface:
                DT.Surface3DPlot(data.X,data.Y,data.C,data.col_headers[i])

            # create a wireframe plot
            if self.do3dWireframe:
                DT.Wireframe3DPlot(data.X,data.Y,data.C,data.col_headers[i])

        report.closeReport()


if __name__ == '__main__':
    CSD = CrossSectionDiagrams('output')
    CSD.colSelector = [20]# range(14,14)
    CSD.CreateDiagrams()
