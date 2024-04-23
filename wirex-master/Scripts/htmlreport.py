# -*- coding: iso-8859-1 -*-
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
# Name:        HTML Report Templates
# Purpose:     Generate HTML based reports for data generated with WireCenter
#
# Author:      asp
#
# Created:     13.08.2015
#-------------------------------------------------------------------------------
#!/usr/bin/env python

import textwrap
import WiPy

#-------------------------------------------------------------------------------
# The following class contains the templates for generating the report in HTML
#-------------------------------------------------------------------------------

class HtmlReport:
    """
    Generate HTML based reports for data generated with WireCenter
    """

    def __init__(self, filename=None):
        """
        Init the report class by setting the constants for header and footer
        """

        self.filename = filename

        self.HTML_HEADER = textwrap.dedent("""
        <html lang="en">
        <head>
        <title>WireCenter Report</title>
        <style type="text/css">
          h1 { color:#179C7D; font-family:Arial; } /* Fraunhofer Gruen #179C7D */
          h2 { color:#000000; background-color:#179C7D; font-family:Arial; }
          p { color:#000000; font-family:Arial; }
          li { color:#000000; font-family:Arial; }
          td { color:#000000; font-family:Arial; }
          li { color:#000000; font-family:Arial; }
          th { color:#000000; font-family:Arial; font-weight:bold; background-color:#DDDDDD;}
          table.page { max-width:100%; border-collapse: collapse; border-style: solid; border-width: 8px; border-color: #ffffff; font-size: 1em;
        	box-shadow: 5px 5px 5px rgba(0, 0, 0, 0.15); -moz-box-shadow: 5px 5px 5px rgba(0, 0, 0, 0.15); -webkit-box-shadow: 5px 5px 5px rgba(0, 0, 0, 0.15); }
        </style>
        </head>
        <body bgcolor="#dddddd">
        <table class="page" width=600 bgcolor="#ffffff" align=center>
        <tr><td>
        <p><img align="right" src="data:image/gif;base64,R0lGODlhuwAeANUhABoaGB
        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
        IZwMk+BA0yfuiOk2mABkrYxmd4MQgAOw=="/></p>
        <h1><a name="top">Report</a></h1>
        <hr>
        """)

        # the end of the html file
        self.HTML_FOOTER = textwrap.dedent("""
        <hr>
        <p>&copy; 2015-2016, Fraunhofer IPA. Report generated with WireCenter Python Toolkit.</p>
        </td></tr></table>
        </body></html>
        """)

        # a list of string containing the content of the html report page
        self.page = []
        
        # a html encoded list of strings with a table of content (and respective
        # cross links)
        self.index = []


    def FigureAsHtml(self,figure):
        """
        This is a helper function to redirect the output of a matplotlib figure
        into an HTML inline encoded png file.
        """
        # load the required packages for printing to a string stream
        import cStringIO
        from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
        # create the required canvas from the figure object
        canvas=FigureCanvas(figure)
        # create a fresh stram object
        stream=cStringIO.StringIO()
        # render the bitmap of the figure into the stream
        canvas.print_png(stream)
        # format the png file in base64 encoding such that is can be embedded into
        # html
        HTML = "<img src=\"data:image/png;base64,{0}\">".format(stream.getvalue().encode("base64").strip())
        # return the generated HTML string
        return HTML

    def newGeneratorInfo(self):
        import time
        import Irobot
        info = "Report generated on " + time.strftime('%X %x') + " using WiPy version " + WiPy.version() + ".<br>\n"
        name, desc, author = Irobot.getMetadata()
        info += "Robot name: " + name + "<br>\n"
        info += "Description: " + desc + "<br>\n"
        self.page.append(info)


    def newRobotGeometry(self):
        """
        Generate a table showing the geometry parameters of the robot in a
        standarized way. Since the interface of WiPy is stateful, the robot 
        to be printed is expetec to be loaded into memory. Thus, the current
        robot geometry data is put into the report as content of a section
        \todo: We may add information on: Number of cables, DOF, motion pattern
        """        
        table = '<table border="1"><tr><td width=50>i</td><td width=250>a<sub>i</sub></td><td width=250>b<sub>i</sub></td></tr>\n'
        
        import Irobot        
        for i in range(0, Irobot.getNow()):
            table += "\t<tr><td>" +  str(i+1) + "</td><td>" + str(Irobot.getBase(i)) + "</td><td>" + str(Irobot.getPlatform(i))+ "</td></tr>\n"
        table += "</table>\n"
        
        self.page.append(table)
        
        
    def newCodeParagraph(self, filename):
        """
        Embed the given file in a source section
        """
        file = open(filename,"r")
        content = file.read()
        self.page.append('<textarea rows="20" wrap="off" readonly style="border:none; width: 100%;">\n')
        self.page.append(content)
        self.page.append('</textarea>\n')


    def newReport(self, filename):
        """
        Start a new report. Store the respective filename in the member of later
        use
        """
        self.filename=filename


    def newSection(self, headline):
        """
        Add a new section <h1> with the given headline
        """
        label = 'l'+str(len(self.page))
        self.page.append('<table width="100%"><tr><td><h1><a name="'+label+'">')
        self.page.append(headline)
        self.page.append('</a></h1></td><td align=right"><a href="#top">top</a></td></table>')
        self.index.append('<li><a href="#'+label+'">'+headline+'</a></li>')


    def newSubSection(self, headline):
        """
        Add a new subsection <h2> with the given headline
        """
        label = 'l'+str(len(self.page))
        self.page.append('<table width="100%"><tr><td><h2><a name="'+label+'">')
        self.page.append(headline)
        self.page.append('</a></h2></td><td align=right"><a href="#top">top</a></td></table>')
        self.index.append('<li><a href="#'+label+'">'+headline+'</a></li>')


    def newParagraph(self, content):
        """
        Add a new text paragraph with the <p> tag
        """
        self.page.append('<p>')
        self.page.append(content)
        self.page.append('</p>')


    def newFigure(self, figure):
        """
        Embedd the given figure into the report using base64 inline encoding
        """
        self.page.append(self.FigureAsHtml(figure))


    def statisticsOverview(self, filename):
        """
        Extract the csv statistics from the html file and embed it into the
        report.
        """
        file = open(filename,"r")
        htmlstats = file.read()
        file.close()
        # extract the table from the file
        start = htmlstats.find('<table>')
        end = htmlstats.find('</table>', start) + 8
        self.page.append(htmlstats[start:end])


    def write(self, content):
        """
        Perform plain writing into the file just putting the content in the file
        """
        self.page.append(content)


    def closeReport(self):
        """
        Generate the file named filename and write the report into that file.
        Use this function to finalize the report.
        """
        file = open(self.filename,'w')
        self.writeReport(file)
        file.close()


    def writeReport(self, file):
        """
        Write the footer and close int the file object (which can be either a
        real file or a StringIO. 
        """
        file.write(self.HTML_HEADER)
        file.write('<h2>Inhalt</h2>')
        for s in self.index:
            file.write(s+'\n')

        for s in self.page:
            file.write(s+'\n')

        file.write(self.HTML_FOOTER)
        file.close()


# selftesting
if __name__ == '__main__':

    WiPy.createRobot()
    report = HtmlReport('report.html')
    report.newSection('Robot Geometry')
    report.newRobotGeometry()
    # setup some sample content
    for i in range(1,3):
        report.newSection('Einleitung'+str(i))
        for j in range(1,4):
            report.newSubSection('Stand der Technik'+str(j))
            report.newParagraph('Some text')
    # finalize the report and write it
    report.closeReport()
    # additionally we store the report in a string and print it to the screen
    import cStringIO
    res = cStringIO.StringIO()
    report.writeReport(res)
    print res.getvalue()
