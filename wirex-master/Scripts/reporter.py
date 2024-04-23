# -*- coding: utf-8 -*-
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
"""
Created on Wed Jul 13 15:11:30 2016

This file is part of the WireCenter Python Toolkit.

This file generates a report on properties of a given robot. Therefore, a 
couple of standard properties of the robot are evaluated and formatted in
diagram, tables, and text sections.
The script can be used in multiple ways including generation of reports in a
web environment, as call-back script from WireCenter, or as stand-alone
command line tool. 

@author: asp
"""

from optparse import OptionParser, OptionGroup
import cStringIO
import WiPy
import wcStl
import htmlreport
import matplotlib
import matplotlib.pyplot as plt
import math
#matplotlib.use('Agg')


class reporter:
    """
    The reporter class is a collection of tools to perform standarized and 
    automated analysis tools for cable robots.
    """

    def __init__(self):
        # use the python general purpose command line parser to get the settings
		# configure command line options and help texts
        self.parser = OptionParser()
        self.parser.add_option("-r", "--robot", # action="store", 
                          type="string", 
                          help="set the robot filename to read robot geometry data in wcrfx")
        self.parser.add_option("-m", "--model", # action="store", dest="model",
                          type="string",
                          help="set the name of the model to be applied for robot geometry generation")                      
        self.parser.add_option("-a", "--algorithm", # action="store", dest="algorithm", 
                          type="string", 
                          help="set the filename to read the algorithm settings in XML")
        self.parser.add_option("-o", "--output", # action="store", dest="output", 
                          type="string",
                          help="if set, the report is written to a file rather than printed to standard out")
        # options to set and change the pose (position and orientation)
        self.posegroup = OptionGroup(self.parser, "Pose definition",
                                     "Options to set position and orientation of the platform for local evaluation")                       
        self.posegroup.add_option("-p", "--position", nargs=3, # dest="pose",
                          type="float", action="callback", callback=self.optparse_Transform,
                          help="set Cartesian position as x y z coordiantes")
        self.posegroup.add_option("--Tx", 
                          action="callback", type="float", callback=self.optparse_Transform,
                          help="apply a translation in x direction to the pose")
        self.posegroup.add_option("--Ty", 
                          action="callback", type="float", callback=self.optparse_Transform,
                          help="apply a translation in y direction to the pose")
        self.posegroup.add_option("--Tz", 
                          action="callback", type="float", callback=self.optparse_Transform,
                          help="apply a translation in z direction to the pose")
        self.posegroup.add_option("--Rx", 
                          action="callback", type="float", callback=self.optparse_Transform,
                          help="apply a rotation [angle in radiant] about the x-axis to the orientation")
        self.posegroup.add_option("--Ry", 
                          action="callback", type="float", callback=self.optparse_Transform,
                          help="apply a rotation [angle in radiant] about the y-axis to the orientation")
        self.posegroup.add_option("--Rz", 
                          action="callback", type="float", callback=self.optparse_Transform,
                          help="apply a rotation [angle in radiant] about the z-axis to the orientation")
        self.parser.add_option_group(self.posegroup)
        
        self.parser.add_option("-l", "--list",
                          action="callback", callback=self.printTests,                             
                          help="list the available tests")
        self.parser.set_usage("%prog [options] test1 test2 ...")
        
        # init the available tests as function dictionary mapping test names 
        # to function pointer
        self.tests = { 
            'hull_cows': self.constantOrientationWorkspace,
            'pose_ik': self.poseInverseKinematics,
            'pose_fd': self.poseForceDistribution,
            'pose_at': self.poseStructureMatrix,
            }

        # fake some opts for testing
        # args = ["-r", "IPAnema.wcrfx", "-a", "settings.xml", "-m", "IPAnema1Design"]

        # execute the command line parsing and store options and arguements (=tests to be executed)
        (self.options, self.args) = self.parser.parse_args()
		# in self.args a list of names is stored that define which tests are performed
		        
        # setup the report object
        self.html = htmlreport.HtmlReport(self.options.output)
		# we want to have partial Rendering here as we want to setup our own page html page template
		# this is currently disabled
        self.html.renderPartial=False

        
    def printTests(self, option, opt, value, parser):
        print "Available tests:"
        print "  ",
        for k in self.tests.keys():
            print k, " ", 
        print


    def optparse_Transform(self, option, opt_str, value, parser):
        """
        Execute the Euclidian rigid body transformations defined by a number of 
        options.
        """
        def matmul(a,b):
            return [[a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
                     a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
                     a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2]],
                    [a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
                     a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
                     a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2]],
                    [a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
                     a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
                     a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2]]]

        # if the values pos and ori do not exist yet generate them with default values
        if not hasattr(parser.values,'pos'):
            setattr(parser.values,'pos',[0,0,0])
#            print "Adding pos: ", parser.values.pos
        if  not hasattr(parser.values,'ori'):
            setattr(parser.values,'ori',[[1,0,0],[0,1,0],[0,0,1]])
            
        # process the possible transformation options
        if opt_str=='--Tx':
            parser.values.pos[0]+=value
        if opt_str=='--Ty':
            parser.values.pos[1]+=value
        if opt_str=='--Tz':
            parser.values.pos[2]+=value
        if opt_str=='--Rx':
            parser.values.ori=matmul(parser.values.ori,[[1,0,0],[0,math.cos(value),-math.sin(value)],[0,math.sin(value),math.cos(value)]])
        if opt_str=='--Ry':
            parser.values.ori=matmul(parser.values.ori,[[math.cos(value),0,math.sin(value)],[0,1,0],[-math.sin(value),0,math.cos(value)]])
        if opt_str=='--Rz':
            parser.values.ori=matmul(parser.values.ori,[[math.cos(value),-math.sin(value),0],[math.sin(value),math.cos(value),0],[0,0,1]])
        if opt_str=='--position':
            parser.values.pos[0]=value[0]
            parser.values.pos[1]=value[1]
            parser.values.pos[2]=value[2]
       
       
    def configure(self):
        # setup the computation model using WiPy
        # load the robot geometry from a file 
        if self.options.robot:
            if (False==Irobot.load(self.options.robot)):
                print "ERROR: could not load robot file ", self.options.robot
                
        # use the algorithm settings provided by a config file; otherwise the default values are used
        if self.options.algorithm:
            if (False==Irobot.loadAlgorithmConfiguration(self.options.algorithm)):
                print "ERROR: could not load algorihm configuration file", self.options.algorithm
        # if
        if self.options.model:
            if (Irobot.applyModel(self.options.model)==False):
                print "ERROR: Invalid model", self.options.model


    def poseInverseKinematics(self):
		"""
		Compute the inverse kinematics for the given pose and print the cable length
		"""
		self.html.newSection("Inverse Kinematics")
		pos = self.options.pos
		self.html.newParagraph("Performing kinematic test for position: r= " + str(pos) + "<br>")
		Ikin.setPosition(*self.options.pos)
		L = Ikin.inverseKinematics2()
		for i,l in zip(range(0,len(L)),L):
		    self.html.newParagraph("Cable Length l_" + str(i+1) + " = " + str(l) + " m<br>")
		fig = plt.figure()
		plt.barh(range(1,Irobot.getNow()+1),L,0.6, align='center')
		plt.xlabel('cable length $l_i$')
		plt.ylabel('cable $i$')
		self.html.newFigure(fig)
#		fig.close()
  

    def poseForceDistribution(self):
		"""
		Compute different force distributions for the given pose
		"""
		self.html.newSection("Force Distribution")
		pos = self.options.pos
		self.html.newParagraph("Performing statics analysis  for position: r= " + str(pos) + "<br>")

		methods = {
			0:"closed Form",
			1:"brute Force",							
			2:"dykstra",							
			3:"weighted Sum", 						
			4:"advanced Closed Form",					
			5:"closed Form Energy Efficient",			
			6:"puncture",							
			7:"advanced Closed Form Energy Efficient",	
			8:"quadratic Programming",				
			9:"wrench Closure"}						
			
		# determines which methods are actually used
		testmethods = [0,2,3,4,5,6,7,8]
		Fall = dict()
		for method in testmethods:
			Iws.setMethod(method,0)
			Fall[method] = Irobot.getForceDistribution(pos[0],pos[1],pos[2],0,0,0)
		
		F=Fall[0]
		self.html.newParagraph("Force f="+str(Fall)+"<br>")
		#for i,f in zip(range(0,len(F)),F):
		#	self.html.newParagraph("Cable force f_" + str(i+1) + " = " + str(f) + " N<br>")
		width = 0.8# / len(testmethods)
		for method in testmethods:
			fig = plt.figure()
			F=Fall[method]
			self.html.newParagraph("Force distribution: "+methods[method])
			plt.barh(range(1,Irobot.getNow()+1),F, width, align='center')

			Frange=Iws.getForceLimits()
			plt.xlim(0,Frange[1]*1.1)
			plt.grid(True)
			plt.axvline(x=Frange[0],color='r')
			plt.axvline(x=Frange[1],color='r')
			plt.text(Frange[0],len(F)+1.2,'$f_{min}$')
			plt.text(Frange[1],len(F)+1.2,'$f_{max}$')
			plt.xlabel('cable force $f_i$')
			plt.ylabel('cable $i$')
			self.html.newFigure(fig)
			plt.close(fig)


    def poseStructureMatrix(self):
        """
        Compute and print the structure matrix for the actual pose.
        """
        self.html.newSection("Structure Matrix")
        pos = self.options.pos
        self.html.newParagraph("Computing structure matrix AT for position: r= " + str(pos))

        Ikin.getStructureMatrix(*pos)
        AT = Ikin.getStructureMatrixMatrix()
        A = map(list, zip(*AT))
        tab = 'AT=<table border="1">\n';
        for col in A:
            tab+="<tr>";
            for cell in col:
                tab+="<td>"+ "{:2.6f}".format(cell) +"</td>"
            tab+="</tr>\n";
        tab+="</table>\n";
        self.html.newParagraph(tab)
        self.html.newParagraph('Singular values of the structure matrix A^T')
        SV = Ikin.getStructureMatrixSingularValues()
        fig = plt.figure()
        plt.barh(range(1,Irobot.getDof()+1),SV, 0.8, align='center')
        plt.xlim(0,SV[1]*1.1)
        plt.grid(True)
        plt.xlabel('$\sigma_i(A^T)$')
        plt.ylabel('i')
        self.html.newFigure(fig)
        plt.close(fig)       
                

    def constantOrientationWorkspace(self):
        """
        Evaluation module for constant orientation workspace
        """
		# configure computation 
        # Iws.setIterations(2)
        Iws.calculateWorkspace()
        Iws.calculateWorkspaceProperties()
        vertex,index = Iws.getWorkspaceGeometry()
        stl = wcStl.wcStl()
        fig = stl.workspaceSTLplot2(vertex,index)  
        self.html.newSection('Constant orientation workspace')
        self.html.newFigure(fig)  
        self.html.newParagraph("Computation time: " + str(Iws.getCalculationTime()) + " &mu;s")  
        self.html.newParagraph("Workspace volume: " + str(Iws.getWorkspaceVolume()) + " m^3")
        self.html.newParagraph("Workspace surface: " + str(Iws.getWorkspaceSurface()) + " m^2")
		
		
    def tester(self):
        """
        Setup the report and execute the tests
        """
        # setup the report file and add the robot description
        self.html.newSection('Robot Geometry')
        self.html.newGeneratorInfo()
        self.html.newRobotGeometry()
        if self.options.robot:
            self.html.newSubSection('Robot Settings')
            self.html.newCodeParagraph(self.options.robot)
        if self.options.algorithm:
            self.html.newSubSection('Algorithm settings')
            self.html.newCodeParagraph(self.options.algorithm)

        if (len(self.args)==0):
            print "WARNING: No test to execute was specified"
        # starting the tests is just looping through the args list and calling 
        # the respective functions. one may even evaluate a test several times
        for test in self.args:
            # try if test exists
            if test in self.tests:
                self.tests[test]()
            else:
                print "WARNING: There is no test " + test + " to be executed."
          
			
    def compileReport(self):
        """
        Finalize the reporting. Write the report to disk or print the 
        information on the screen
        """
        if self.options.output:
            self.html.closeReport()
        else:
            # generate the report into the string
            res = cStringIO.StringIO()
            self.html.writeReport(res)
            print res.getvalue()


    def main(self):
        """
        The basic workflow 
        """
        self.configure()
        self.tester()
        if len(self.args)>0:
            self.compileReport()

    
if __name__ == '__main__':
    # execute the report if the script is called from command line with args
    report = reporter()
    report.main()
