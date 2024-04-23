import Irobot
import Iapp

def main():
	print 'Tests fuer Python Scripting'
	Irobot.load('IPAnema.txt')
	Irobot.setIterations(4)
	Irobot.setEps(0.001)
	Irobot.setWrench(0,0,-50,0,0,0)
	Irobot.calculateWorkspaceCrosssection('y')
	Irobot.calculateWorkspaceProperties()
	Irobot.printProperties()
	Irobot.saveWorkspaceCrosssection('IPAnema.svg','y')
	
	Irobot.calculateWorkspace()
	Irobot.calculateWorkspaceProperties()
	Irobot.printProperties()
	Irobot.saveWorkspace('Kaercher_ws.stl')
	Iapp.invalidate()
	eps = 0.1
	deltax = 0.0
	deltay = 0.5
	zmin = 0
	deltaz = 0.8
	while deltax < 2:
		if (Irobot.verifyWorkspaceBox(-deltax,-deltay,zmin,deltax,deltay,zmin+deltaz,eps,eps,eps)!=0):
			print 'py: Workspace Test good. deltax=', deltax
		else:
			print 'py: Workspace Test failed. deltax=', deltax
		deltax+=0.05
	print 'py: Call is done.'
