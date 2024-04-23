from IWC import *

def main():

	# Build the robot
	main12()
		
	Irobot.setProjectionCenter(0,0,0.5)
	Irobot.setSearchRange(10)
	Irobot.setEps(0.0001)
	Irobot.setIterations(6)
	Irobot.setWrench(0,0,0,0,0,0)
	Irobot.setForceLimits(50,720)

	# now we look through a couple of orientations and calculate the workspace
	for phi in range(1,1):
		
		Irobot.setOrientation(0,0,phi*10*2*3.1415926/360.0)
		Irobot.calculateWorkspace()
		Irobot.calculateWorkspaceProperties()
		pic = "pic{0}.bmp".format(phi)
		Iapp.invalidate()
		Iapp.saveBmp(pic,1024,768)
		
	return

def main9():
	Irobot.load("IPAnema9.txt")
	R=1
	r=0.1
	H2 = 1
	H1 = 0.5
	h2 = 0.25
	h1 = 0.2
	cos30 = 0.8660254
	sin30 = 0.5

	# set the parameters of the frame
	Irobot.setBase(0, 0, R, H2)
	Irobot.setBase(1, cos30*R, -sin30*R, H2)
	Irobot.setBase(2, -cos30*R, -sin30*R, H2)

	Irobot.setBase(3, -cos30*R, sin30*R, H1)
	Irobot.setBase(4, cos30*R, sin30*R, H1)
	Irobot.setBase(5, 0, -R, H1)

	Irobot.setBase(6, 0, R, 0)
	Irobot.setBase(7, cos30*R, -sin30*R, 0)
	Irobot.setBase(8, -cos30*R, -sin30*R, 0)
		
	# set the parameters of the end-effector
	Irobot.setPlatform(0, r, 0,  h2)
	Irobot.setPlatform(1, r, 0,  h2)
	Irobot.setPlatform(2, r, 0,  h2)
	Irobot.setPlatform(3, 0, 0,  h1)
	Irobot.setPlatform(4, 0, 0,  h1)
	Irobot.setPlatform(5, 0, 0,  h1)
	Irobot.setPlatform(6, 0, 0,  0)
	Irobot.setPlatform(7, 0, 0,  0)
	Irobot.setPlatform(8, 0, 0,  0)

	return

def main12():
	Irobot.load("IPAnema12.txt")
	R=6
	r=0.5
	H2 = 10
	H1 = 9
	h2 = 1.8
	h1 = 1.5

	# set the parameters of the frame
	Irobot.setBase(0, -R,  R, H2)
	Irobot.setBase(1,  R,  R, H2)
	Irobot.setBase(2,  R, -R, H2)
	Irobot.setBase(3, -R, -R, H2)

	Irobot.setBase(4, -R,  R, H1)
	Irobot.setBase(5,  R,  R, H1)
	Irobot.setBase(6,  R, -R, H1)
	Irobot.setBase(7, -R, -R, H1)

	Irobot.setBase(8, -R,  R, 0)
	Irobot.setBase(9,  R,  R, 0)
	Irobot.setBase(10,  R, -R, 0)
	Irobot.setBase(11, -R, -R, 0)
		
	# set the parameters of the end-effector
	Irobot.setPlatform(0, r, 0,  h2)
	Irobot.setPlatform(1, r, 0,  h2)
	Irobot.setPlatform(2, r, 0,  h2)
	Irobot.setPlatform(3, r, 0,  h2)
	Irobot.setPlatform(4, 0, 0,  h1)
	Irobot.setPlatform(5, 0, 0,  h1)
	Irobot.setPlatform(6, 0, 0,  h1)
	Irobot.setPlatform(7, 0, 0,  h1)
	Irobot.setPlatform(8, 0, 0,  0)
	Irobot.setPlatform(9, 0, 0,  0)
	Irobot.setPlatform(10, 0, 0,  0)
	Irobot.setPlatform(11, 0, 0,  0)

	return

