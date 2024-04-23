# -*- coding: iso-8859-1 -*-
# this file contains some scripts which are automatically executed
# at the time of special events of the application such as
# OnStart, OnKinematicUpdate
# (c)opyright 2009-2018 by Andreas Pott, Fraunhofer IPA

from IWC import *
import os

def OnStart():
	return

# pose update is executed whenever new  position and orientation data are
# applied to the platform. pose update should perform pose dependent analysis
# functions.
def OnUpdatePose():
	#print "Py: Update Pose"
	return

def OnHelp():
	print "Methods of the Irobot object"
	print "----------------------------"
	Irobot.help()
	print " "
	print "Methods of the Iapp object"
	print "--------------------------"
	Iapp.help()


# define your own action as you like; just start their names with prefix action... 
# all functions with other names are considered to be private.
def actionDefault():
    print "mostly harmless"
    

def actionReportCows():
    """
    Generate a report on the constant orientation workspace (cow) using WiPy toolset 
    and store it in the predefined filename
    report.html to automatically show in wirecenter report view
    """
    Irobot.saveAlgorithmConfiguration("_alg.xml")
    Irobot.save("_robot.wcrfx")
    os.system("python reporter.py -r _robot.wcrfx -a _alg.xml -o report.html hull_cows")
    os.remove("_robot.wcrfx")
    os.remove("_alg.xml")

    print "Cows here"


def actionReportStatics():
    """
    Generate a report on the pose using WiPy toolset and store it in the 
    predefined filename
    report.html to automatically show in wirecenter report view
    """
    Irobot.saveAlgorithmConfiguration("_alg.xml")
    Irobot.save("_robot.wcrfx")

    pos = Ikin.getPosition()
    pos_str = str(pos[0])+" "+str(pos[1])+ " "+str(pos[2])
    cmd = "python reporter.py -r _robot.wcrfx -a _alg.xml -p "+pos_str+" -o report.html pose_fd"
    os.system(cmd)    

    os.remove("_robot.wcrfx")
    os.remove("_alg.xml")
    