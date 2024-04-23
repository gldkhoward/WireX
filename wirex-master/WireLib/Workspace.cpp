/*
* WireX  -  WireLib
*
* Copyright (c) 2006-2019 Andreas Pott
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

/*!*******************************************************************
 *  \file   : Workspace.cpp
 *
 *  Project   Wirelib
 *
 *  \Author   Andreas Pott
 *
 *  \Date     10.10.2008 (part of MoWiRoGeometry)
 *			  15.12.2009 (refactored by asp)
 *
 *********************************************************************
 */ 

#include "Workspace.h"
#include "Kinematics.h"
#include <time.h>

namespace PCRL {

CWorkspaceAlgorithm::CWorkspaceAlgorithm(CRobotData &robot)
	: CAlgorithm(robot)
{
	center=Vector3d(0,0,1);
	OrientationSet.push_back(Matrix3d::ZRotationMatrix3d(0.0));
	// algorithm parameter
	IterationDepth=3;
	eps=0.0001;
	searchMax = 12;
	allOrientations=true;
	maxBB = minBB = Vector3d(0,0,0);
	method = forceFeasible;
	calculationTime = 0;

	pForceDistribution = 0;
	pKinematics = 0;		
	pWrenchSet = 0;
	pVelocitySet = 0;
	pStiffness = 0;
}

void CWorkspaceAlgorithm::bind()
{
	CAlgorithm::bind();
	if (!pReflector)
		return;
	pReflector->bind(IterationDepth,"IterationDepth","Workspace/@IterationDepth");
	pReflector->bind(eps,"eps","Workspace/@eps");
	pReflector->bind(searchMax,"searchMax","Workspace/@searchMax");
	pReflector->bind(allOrientations,"allOrientations","Workspace/@allOrientations");
	pReflector->bind((int&)method,"method","Workspace/@method");
	pReflector->bind(eps,"eps","Workspace/@eps");
	pReflector->bind(maxBB.x(),"maxBB.x","Workspace/BoundingBox/@max.x");
	pReflector->bind(maxBB.y(),"maxBB.y","Workspace/BoundingBox/@max.y");
	pReflector->bind(maxBB.z(),"maxBB.z","Workspace/BoundingBox/@max.z");
	pReflector->bind(minBB.x(),"minBB.x","Workspace/BoundingBox/@min.x");
	pReflector->bind(minBB.y(),"minBB.y","Workspace/BoundingBox/@min.y");
	pReflector->bind(minBB.z(),"minBB.z","Workspace/BoundingBox/@min.z");
}

/*! Driver function to select between different evaluation criteria (force 
 *  distribution, wire length, ...).
 *  Apply the respective workspace test to the pose given by po, Orientation
 *  \param p0 [in] The position that is tested
 *  \param Orientation [in] The orientation that is tested
 *  \return true, if the assessed workspace test was postively evaluted 
 */
bool CWorkspaceAlgorithm::testPose(const Vector3d& p0, const Matrix3d& Orientation)
{
	// select the resprective workspace test based on the state variable "method"
	switch (method)
	{
	case forceFeasible:
		return pForceDistribution->testPose(p0,Orientation);
	case wireLength:
		return pKinematics->testPose(p0,Orientation);
	case boundingBox:	// experimental; allow only positions inside a box with vertices minBB and maxBB
		if (p0.x() > minBB.x() && p0.x() < maxBB.x() &&
			p0.y() > minBB.y() && p0.y() < maxBB.y() &&
			p0.z() > minBB.z() && p0.z() < maxBB.z())
			return true;
		else
			return false;
	case wrenchSet:
		return pWrenchSet->testPose(p0,Orientation);
	case velocitySet:
		return pVelocitySet->testPose(p0,Orientation);
	case stiffness:
		return pStiffness->testPose(p0,Orientation);
	default:
		return false;
	}
}

/*! Driver to check for one or all orientations in the orientation set
 *  This function is part of CWorkspaceAlgorithms implementation to deal
 *  with different types of workspace, namely with 
 *  - constant orientation workspace if only sizeof(orientationset)==1
 *  - including orientation workspace if allOrientation==False and
 *    sizeof(orientationset)>1
 *  - total orientation workspace if allOrientation==True and 
 *    sizeof(OrientationSet)>1
 *  \param p0 [in] the position to be used for the workspace evaluation
 *  \return true, if the position belongs to the workspace.
 */
bool CWorkspaceAlgorithm::workspaceEvaluator(const Vector3d& p0)
{
	if (allOrientations)
	{
		for (unsigned int i=0; i<OrientationSet.size(); i++)
		{
			if (!testPose(p0,OrientationSet[i]))
				return false;
		}
		return true;
	}
	else
	{
		for (unsigned int i=0; i<OrientationSet.size(); i++)
		{
			if (testPose(p0,OrientationSet[i]))
				return true;
		}
		return false;
	}
}

/*! perform a line search along the vector v starting from center
 *  The minimum value of the line search is given by lambda_min
 *  and the maximum value is lambda_max.
 *  The bisection for the line search is stopped if the difference
 *  between min and max is <eps. The generic driver function workspaceEvaluator 
 *  is called and check to be true. The whole range of orientations is passed to 
 *  for the current orientation to evaluate if a pose belongs to the workspace.
 */
void CWorkspaceAlgorithm::lineSearch(Vector3d& v, double lambda_min, double lambda_max)
{
	// normalize the search direction
	Vector3d v0=v-center;
	v0.normalize();
  
	double lambda=lambda_max;
    while (lambda_max-lambda_min>eps)
	{
        Vector3d p0 = center + lambda*v0;
        if (workspaceEvaluator(p0))
            lambda_min=lambda;
        else
            lambda_max=lambda;
        lambda=(lambda_min+lambda_max)/2;
    }
	v = center + lambda*v0;
}

/*! perform a line search approach to query the orientational workspace where
 *  \param r [in] the position where to perform the evaluation
 *  \param u [in] is the used rotational axis
 *  \param lambda_min [in] is the minimal angle for the search interval
 *  \param lambda_max [in] is the maximum angle for the serach interval
 */
double CWorkspaceAlgorithm::lineSearchAngular(const Vector3d& r, const Vector3d& u, double lambda_min, double lambda_max)
{
	// firstly we have to normalize the rotation axis
	Vector3d U = u;
	U.normalize();
	// init the search range
	double lambda=lambda_max;
    while (lambda_max-lambda_min>eps)
	{
		Matrix3d Ri = Matrix3d::Identity();
		getMatrixFromAxisAngle(Ri,lambda,U.x(),U.y(),U.z());
		if (testPose(r,Ri))
            lambda_min=lambda;
		else
            lambda_max=lambda;
        lambda=(lambda_min+lambda_max)/2;
    }
	// on exit the latest value of lambda is kept and therefore reported to the caller
	return lambda;
}


/*! calculate the aperture of the cone around axis u and apex at a_i that 
 *  fully covers a workspace box given through (min,max) where i selects the i-th winch
 *  return maximum apperture or a negative value on failure
 */
double CWorkspaceAlgorithm::getWinchAperture(const int& i, const Vector3d& Min, const Vector3d& Max, const Vector3d& u)
{
	// from mathematical considerations about sperical geometry it seems (not proven)
	// that the maximum angles occures between the axis u and the vertices of the box
	// given by Min/Max if the base point a_i is not inside the box. 
	Matrix3d Orientation = OrientationSet[0];

	// firstly check if a_i is inside the range given through (Min/Max) translated by b_i and rotated by Orientation
	Vector3d u0=u/u.norm();
	double angle = 0 * DEG_TO_RAD;
	Vector3d l;
	l = (Min+Orientation*pRobot->getPlatform(i)-pRobot->getBase(i));							        l.normalize(); angle = max(angle, acos(u0.dot(l)));
	l = (Vector3d(Max.x(),Min.y(),Min.z())+Orientation*pRobot->getPlatform(i)-pRobot->getBase(i));	l.normalize(); angle = max(angle, acos(u0.dot(l)));
	l = (Vector3d(Min.x(),Max.y(),Min.z())+Orientation*pRobot->getPlatform(i)-pRobot->getBase(i));	l.normalize(); angle = max(angle, acos(u0.dot(l)));
	l = (Vector3d(Max.x(),Max.y(),Min.z())+Orientation*pRobot->getPlatform(i)-pRobot->getBase(i));	l.normalize(); angle = max(angle, acos(u0.dot(l)));
	l = (Vector3d(Min.x(),Min.y(),Max.z())+Orientation*pRobot->getPlatform(i)-pRobot->getBase(i));	l.normalize(); angle = max(angle, acos(u0.dot(l)));
	l = (Vector3d(Max.x(),Min.y(),Max.z())+Orientation*pRobot->getPlatform(i)-pRobot->getBase(i));	l.normalize(); angle = max(angle, acos(u0.dot(l)));
	l = (Vector3d(Min.x(),Max.y(),Max.z())+Orientation*pRobot->getPlatform(i)-pRobot->getBase(i));	l.normalize(); angle = max(angle, acos(u0.dot(l)));
	l = (Max+Orientation*pRobot->getPlatform(i)-pRobot->getBase(i));							        l.normalize(); angle = max(angle, acos(u0.dot(l)));

	return angle;
}

//! copy algorithm parameters from another CWorkspaceAlgorithm instance. 
//! \param src [in] reference to the workspace objects which settings shall be copied.
void CWorkspaceAlgorithm::getAlgorithmParameter(const CWorkspaceAlgorithm& src)
{
	center = src.center;
	OrientationSet = src.OrientationSet;
	eps = src.eps;
	IterationDepth = src.IterationDepth;
	searchMax = src.searchMax;
	allOrientations = src.allOrientations;
	method = src.method;
	minBB = src.minBB;
	maxBB = src.maxBB;
	center = src.center;
}

/*! create a set of orientations "around" the given orientation R0
 *  Bryant angles are used to variate the orientation in a range of [-delta_a...delta_a]
 *  The results is multiplied by R0.
 *  if the values passed to delta_a, delta_b, delta_c are not positive, the repsective
 *  direction is skipped when creating the grid
 *  By default 10 variations are performed for each directions.
 *  \param delta_a [in] range around the x-axis in radiant
 *  \param delta_b [in] range around the y-axis in radiant
 *  \param delta_c [in] range around the z-axis in radiant
 *  \param steps [in] number of variations per axis; by default 10 steps are generated
 *  \param R0 [in] base orientation. The set is constructed around this matrix; by default identity matrix
 *  \return true, if the set was successfully generated
 */
bool CWorkspaceAlgorithm::createOrientationSet(double delta_a, double delta_b, double delta_c, const unsigned int& steps, const Matrix3d& R0)
{
	// delete the current orientation set
	OrientationSet.clear();
	// check for valid parameters
	if (delta_a<0)
		delta_a=0;
	if (delta_b<0)
		delta_b=0;
	if (delta_c<0)
		delta_c=0;
	double a=0,b=0,c=0;
	for (a=-delta_a; a<=delta_a; a+=(delta_a>0 ? 2*delta_a/steps : 1))
		for (b=-delta_b; b<=delta_b; b+=(delta_b>0 ? 2*delta_b/steps : 1))
			for (c=-delta_c; c<=delta_c; c+=(delta_c>0 ? 2*delta_c/steps : 1))
				addOrientation(R0*Matrix3d::XRotationMatrix3d(a)*Matrix3d::YRotationMatrix3d(b)*Matrix3d::ZRotationMatrix3d(c));
	return true;
}

/*! do the same as CWorkspaceAlgorithm::createOrientationSet, considering delta_x=delta_a=delta_b=delta_c
 *  \param delta_x [in] range around the x-,y- and z-axis in radiant
 *  \param steps [in] number of variations per axis; by default 10 steps are generated
 *  \param R0 [in] base orientation. The set is constructed around this matrix; by default identity matrix
 *  \return true, if the set was successfully generated
 */
bool CWorkspaceAlgorithm::createSymmetricOrientationSet(double delta_x, const unsigned int& steps, const Matrix3d& R0)
{
	// delete the current orientation set
	OrientationSet.clear();
	// check for valid parameters
	if (delta_x<0)
		delta_x=0;
	double a=0,b=0,c=0;
	for (a=-delta_x; a<=delta_x; a+=(delta_x>0 ? 2*delta_x/steps : 1))
		for (b=-delta_x; b<=delta_x; b+=(delta_x>0 ? 2*delta_x/steps : 1))
			for (c=-delta_x; c<=delta_x; c+=(delta_x>0 ? 2*delta_x/steps : 1))
				addOrientation(R0*Matrix3d::XRotationMatrix3d(a)*Matrix3d::YRotationMatrix3d(b)*Matrix3d::ZRotationMatrix3d(c));
	return true;
}

/*!
 * Fill the orientation set with random values, possible following the restriction given by R0 and max_angle
 * \remark: Note that the method is rather inefficient if the max_angle parameter is small as a huge number
 *          of orientations is discarded. 
 * \param count [in] The number of random orienations to be added
 * \param R0 [in] a reference orientation. Only meaningful if max_angle in [0, 2*PI [ , 
		  i.e. if the angle restricts the range where orientation shall be generated
 * \param max_angle [in] All generated orientations which distance to the reference 
		  R0 is larger than max_angle are discarded
 * \return true, if successful
 */
bool CWorkspaceAlgorithm::createRandomOrientationSet(const int count, Matrix3d& R0, const double& max_angle)
{
	// check parameter
	if (count <= 0)
		return false;
	// we cannot handle a negative distance or if the offset is below the threshold 
	if (max_angle <= 1e-3)
		return false;
	OrientationSet.clear();
	Matrix3d R;
	while ((int)OrientationSet.size() < count)
	{
		getMatrixRandomOrientation(R);
		// max_angle is default value or larger?
		if (max_angle >= 2*MO_PI)
			addOrientation(R);
		else
			// test if the generated orientation as close to reference as desired
			if (getSO3Distance(R0, R) < max_angle)
				addOrientation(R);
	}
	return true;
}

void CWorkspaceAlgorithm::setBoundingBox(const Vector3d& MinBB, const Vector3d& MaxBB)
{
	// check the range before assigning the values to verify that minBB < maxBB hold
	minBB.x() = min(MinBB.x(),MaxBB.x());
	minBB.y() = min(MinBB.y(),MaxBB.y());
	minBB.z() = min(MinBB.z(),MaxBB.z());
	maxBB.x() = max(MinBB.x(),MaxBB.x());
	maxBB.y() = max(MinBB.y(),MaxBB.y());
	maxBB.z() = max(MinBB.z(),MaxBB.z());
}

//////////////////////////////////////////////////////////////////////////////
// CWorkspaceHull
//////////////////////////////////////////////////////////////////////////////

CWorkspaceHull::CWorkspaceHull(CRobotData& robot) : CWorkspaceAlgorithm(robot)
{
	FinishedTriangles = 0;
	currentVertex = 0;
	currentTriangle = 0;
	roiStart = 0;
	roiEnd = 0;

	Surface = 0;
	Volume = 0;
}

CWorkspaceHull::~CWorkspaceHull()
{
}

/*! \brief This function calculates properties of the workspace:
 *  volume, surface, center of gravity, and an axis-aligned bounding box
 * 
 */
bool CWorkspaceHull::calculateWorkspaceProperties()
{
	// calcualte the volume, the area of the surface and the center of gravity of the workspace
	Volume = 0;
	Surface = 0;
	CoI = Vector3d(0,0,0);	// estimate the center of gravity for the workspace
	return calculateWorkspaceProperties(Volume, Surface, CoI);
}


/*! \brief This function calculates properties of the workspace:
 *  volume, surface, center of gravity, and an axis-aligned bounding box
 * 
 */
bool CWorkspaceHull::calculateWorkspaceProperties(double &Volume, double &Surface, Vector3d &CoI)
{
	unsigned int i;
	for (i=0; i<Triangles.size(); i++)
	{
		// calculate the volume of one tetraeder
		Vector3d a = vertices[Triangles[i].i]-center;
		Vector3d b = vertices[Triangles[i].j]-center;
		Vector3d c = vertices[Triangles[i].k]-center;
		double vi = (a.cross(b).dot(c))/6.0;
		Volume += vi;  
		// calculate the center of gravity
		CoI += vi*(vertices[Triangles[i].i]+vertices[Triangles[i].j]+vertices[Triangles[i].k]+center)/4;
		// calculate the surface of one triangle
		Surface += 0.5*((a-b).cross(a-c)).norm(); 
	}
	if (Volume > 0)
	{
		CoI = CoI/Volume;
		center = CoI;
	}

	// calculate the axis aligned bounding box of the workspace
	if (vertices.size())	// if there are vertices, use the first one as initial estimate for the bounding box
	{
		bbMin = vertices[0];
		bbMax = vertices[0];
	}
	for (i=0; i<vertices.size(); i++)
	{
		if (bbMin.x()>vertices[i].x()) bbMin.x() = vertices[i].x();
		if (bbMin.y()>vertices[i].y()) bbMin.y() = vertices[i].y();
		if (bbMin.z()>vertices[i].z()) bbMin.z() = vertices[i].z();
		if (bbMax.x()<vertices[i].x()) bbMax.x() = vertices[i].x();
		if (bbMax.y()<vertices[i].y()) bbMax.y() = vertices[i].y();
		if (bbMax.z()<vertices[i].z()) bbMax.z() = vertices[i].z();
	}

	return true;
}

/*! print the calculated properties on the screen
 */
void CWorkspaceHull::printProperties() const
{
	cout << "Number of Triangles:      " << Triangles.size() << endl; 
	cout << "Volume of the workspace:  " << Volume << " m^3\n";
	cout << "Surface of the workspace: " << Surface << " m^2\n";
	cout << "center of gravity:        " << CoI << " m\n";
	cout << "Calculation time:         " << calculationTime << " ms\n";
	cout << "Bounding box min:         " << bbMin << " m\n";
	cout << "Bounding box max:         " << bbMax << " m\n";
}

//! the follwing computations are only a subset of the calculate workspace;
//! the core algorithm leaves out the generation of the sphere
//! and does not care about computation time determination.

bool CWorkspaceHull::calculateWorkspaceCore(bool initializedByLast)
{
	// check if region of interest (roi) is feasible
	if (roiStart<0 || roiStart>vertices.size() || roiEnd<0 || roiEnd>vertices.size())
		return false;

	// loop through all vertices 
	unsigned int i;

	if (!initializedByLast)
	{
		for (i=roiStart; i<roiEnd; i++)
			lineSearch(vertices[i],0.0,searchMax);
	}
	else
	{
		// use vertices for lower and upper lambda_min, lambda_max
		
		Vector3d v0;
		double l;
		for (i=roiStart; i<roiEnd; i++)
		{
			v0=vertices[i]-center;
			l = v0.norm();
			lineSearch(vertices[i],l-0.1,l+0.1);
		}
	}

	return true;
}


//! the following functions are the refactored versions
bool CWorkspaceHull::calculateWorkspace(bool initializedByLast)
{
	// start stopwatch
	time_t start=clock();
	
	if (!initializedByLast) // use last hull as initial guess (maybe useful for iterative algorithms)
	{
		// generate a new grid from a unit sphere where the accuracy depends on the current iteration value
		makeSphere();
	}
	
	// firstly we evaluate the center point since it tells us importing things if it makes sence to consider the hull
	bCenterValid = testPose(center,OrientationSet[0]);

	// call the core function for workspace generation
	if (!calculateWorkspaceCore(initializedByLast))
		return false;

	FinishedTriangles=Triangles.size();

	// stop stopwatch
	calculationTime = (int)(clock()-start);

	return true;
}

//! load a CSV file with one vertice per line. 
// the Iteration depth has to be preconfigured using the standard dialog box for workspace calculations
bool CWorkspaceHull::loadWorkspaceCSV(const string& filename, int iIterationDepth)
{
	IterationDepth = iIterationDepth;
	
	// generate a new grid from a unit sphere where the accuracy depends on the current iteration value
	makeSphere();
	
	unsigned int i=0;

	string word = ""; // string value for the separation operator like ;
	ifstream in;
	in.open (filename);
	while (in >> vertices[i].x() >> word >> vertices[i].y()>> word >> vertices[i].z()>> word)
	{
		i++;
		if (i>= vertices.size()) 
			break;
	}

	FinishedTriangles=Triangles.size();

	return true;
}


/*! recalcuate the workspace with the current settings and perform the boolean
 *  AND operation with the workspace that is internally stored (e.g. from 
 *  previous calls to calculateWorkspace)
 */
bool CWorkspaceHull::intersectWorkspace()
{
	if (vertices.size()<=0)
		return false;

	// start stopwatch
	time_t start=clock();

	// check if region of interest (roi) is feasible
	if (roiStart<0 || roiStart>vertices.size() || roiEnd<0 || roiEnd>vertices.size())
		return false;

	// loop through all vertices and check for smaller bounds
	unsigned int i;
	for (i=roiStart; i<roiEnd; i++)
		lineSearch(vertices[i],0,vertices[i].norm());

	// stop stopwatch
	calculationTime = (int)(clock()-start);

	return true;
}

/*! recalcuate the workspace with the current settings and perform the boolean
 *  OR operation with the workspace that is internally stored (e.g. from 
 *  previous calls to calculateWorkspace)
 */
bool CWorkspaceHull::uniteWorkspace()
{
	if (vertices.size()<=0)
		return false;

	// start stopwatch
	time_t start=clock();
	
	// check if region of interest (roi) is feasible
	if (roiStart<0 || roiStart>vertices.size() || roiEnd<0 || roiEnd>vertices.size() || roiStart < roiEnd)
		return false;

	// loop through all vertices and check for larger bounds
	unsigned int i;
	for (i=roiStart; i<roiEnd; i++)
		lineSearch(vertices[i],vertices[i].norm(),searchMax);

	// stop stopwatch
	calculationTime = (int)(clock()-start);

	return true;
}

//! set the region of interest w.r.t. to the vertex data, i.e. the index-range to be checked 
//! in workspace calculation
bool CWorkspaceHull::setVertexRoi(const unsigned int& start, const unsigned int& end)
{
	// check feasibility of parameter
	if (start>end || start < 0 || end > vertices.size())
		return false;
	roiStart = start;
	roiEnd = end;
	return true;
}

//! get the vector from center to the hull surface
//! if the desired index is invalid we return a 0-vector
Vector3d CWorkspaceHull::getVertexRay(const unsigned int& id)
{
	if ( id < vertices.size() )
		return vertices[id]-center;
	return Vector3d(0,0,0);
}

//! get the length of the line from center to the workspace hull
double CWorkspaceHull::getVertexRayLength(const unsigned int& id)
{
	return sqrt(getVertexRay(id).squaredNorm());
}

//! helper functions to enable parallel prozessing of workspace
//! copy vertex data from one object to another
bool CWorkspaceHull::copyVertexData(const CWorkspaceHull& src, const unsigned int& start, const unsigned int& end)
{
	// check feasibility of parameter
	if (start>end || start < 0)
		return false;
	// check if the range is covered by both this object and the source object
	if (vertices.size() < end || src.vertices.size() < end)
		return false;
	// copy the vertex data
	for (unsigned int i=start; i<end; i++)
		vertices[i] = src.vertices[i];
	return true;
}

/*! Computes optimal cone for a given apex
 * \param centerLine: reference to a 3x1 vector that is used to store the center line vector
 * \param angle: reference to a variable  that is used to store the angle between the cone's center line
 *				 and surface
*/
void CWorkspaceHull::calculateOptimalCone(const Vector3d& apex, Vector3d& axis_out , double& aperture_out)
{
	vector<unsigned int> vID;
	vector<unsigned int> bID;
	
	/*
	if (vertices.size() == 0)
	{
		vertices.push_back(Vector3d(1,1,3));
		vertices.push_back(Vector3d(1,1,2));
		vertices.push_back(Vector3d(1,1.5,0));
		vertices.push_back(Vector3d(2,2,2));
	}*/

	// initialize vID list;
	for(unsigned int i = 0; i < vertices.size(); i++)
	{
		vID.push_back(i);
	}
	
	calculateOptimalCone_(vID, bID, apex, axis_out , aperture_out);	
	
	// Here aperture is define as the angle between the center line and the cone surface
	aperture_out = aperture_out/2;
	axis_out.normalize();
	axis_out = axis_out*4;

	printf("Optimal cone for apex (%f, %f, %f):\n", apex.x(), apex.y(), apex.z());
	printf("Axis: (%f,%f,%f)\n",axis_out.x(), axis_out.y(), axis_out.z());
	printf("Aperture (rad): (%f)\n",aperture_out);
}

//! Computes the optimal cone for a given id of the robot base
void CWorkspaceHull::calculateOptimalCone(const unsigned int id, Vector3d& axis_out , double& aperture_out)
{
	calculateOptimalCone(pRobot->getBase(id), axis_out, aperture_out);
}

void CWorkspaceHull::calculateOptimalCone_(vector<unsigned int> vID, vector<unsigned int> bID, const Vector3d& apex, Vector3d& axis_out , double& aperture_out)
{

	if (vID.empty() || bID.size()== 3)
	{
		switch(bID.size())
		{
		case 0:	// return empty cone
			{
			axis_out = Vector3d(1,0,0);
			aperture_out = 0;
			break;
			}
			
		case 1: // return minimal cone for one point
			{
			axis_out = vertices[bID[0]]-apex;
			aperture_out = 0;
			break;
			}
			
		case 2: // return minimal cone for two points
			{
			Vector3d v1 = vertices[bID[0]]-apex;
			Vector3d v2 = vertices[bID[1]]-apex;
			v1.normalize();
			v2.normalize();
			axis_out =  v1 + v2;
			aperture_out = atan2(v1.cross(v2).norm(),v1.dot(v2));
			break;
			}

		case 3: // return minimal cone for three points
			{
			Vector3d p1 = vertices[bID[0]] - apex;
			Vector3d p2 = vertices[bID[1]] - apex;
			Vector3d p3 = vertices[bID[2]] - apex;

			p1.normalize();
			p2.normalize();
			p3.normalize();

			Vector3d u = p2-p1;
			Vector3d v = p3-p1;
			Vector3d w = p3-p2;
			Vector3d c;
			c = u.cross(v);
			double u2 = u.dot(u);
			double v2 = v.dot(v);
			double w2 = c.dot(c);
			if (w2 < 0.00001) w2 = 0.00001; // avoid zero division
			axis_out = p1+(u2*(v.dot(w)*v)-v2*(u.dot(w)*u))/(2*w2);
			Vector3d c2;
			c2 = axis_out.cross(p1);
			aperture_out = 2*atan2(c2.norm(),axis_out.dot(p1));
			break;
			}
		}

	}
	else
	{
		unsigned int i, p_i;
		i = (unsigned int)ceil((vID.size()-1.0)* ((double)rand()/RAND_MAX));
		p_i = vID[i];
		vID.erase(vID.begin()+i);

		calculateOptimalCone_(vID,bID,apex,axis_out,aperture_out);

		if(!isInsideCone(p_i,apex,axis_out,aperture_out))
		{
			bID.push_back(p_i);
			calculateOptimalCone_(vID,bID,apex,axis_out,aperture_out);
		}		
	}
}

bool CWorkspaceHull::isInsideCone(const unsigned int& p_i, const Vector3d& apex, const Vector3d& axis , const double& aperture)
{
	Vector3d rp = vertices[p_i] - apex;
	Vector3d c;
	c = axis.cross(rp);
	double phi = atan2(c.norm(),axis.dot(rp));
	return (abs(phi) <= aperture *0.5);	
}


/*
 * Bloat the workspace hull by a given deltaLength [m] in each direction of
 * the vertices that is it shifts the point of each vertices to be deltaLength
 * further away from the center
 */
bool CWorkspaceHull::bloatHullBy(double deltaLength)
{
	unsigned int i;

	//! For every vertice we have already defined
	for ( i = 0; i < vertices.size(); i++ )
	{
		//! Get the vector pointing from the center to the vertices
		Vector3d v3dVector = vertices[i] - center;
		//! Keep the vector we will add here
		Vector3d v3dDeltaVec = Vector3d(0, 0, 0);

		//! Normalize vector to length 1 (ONE)
		v3dVector.normalize();

		//! Create the new vector by using the old vector and adding a vector of
		//! length deltaLength and orientation of v3dVector
		v3dDeltaVec = v3dVector*deltaLength;
		
		//! Check if the scalar product of the vectors is not negative and the
		//! length of the delta vector is not greater than the actual length of
		//! the vector
		if ( v3dDeltaVec.norm() > v3dVector.norm() && v3dDeltaVec.dot(v3dVector) < 0  )
		{
			//! If so, set the delta vector to the negative original vector
			//! which will result in the vertices being 0 (all zero)
			v3dDeltaVec = -v3dVector;
		}

		//! And update that vertices vector entry
		vertices[i] += v3dDeltaVec;
	}

	return true;
}


//! compute the workspace taking into account collisions with a axi aligned bounding box
//! given by vBox_min and vBox_max. the routine will shring an existing workspace. 
//! \todo This collision algorithm is somewhat misplaced in this class. 
void CWorkspaceHull::calcDisabledWspc(Vector3d& vBox_min, Vector3d& vBox_max)
{
	unsigned int i;
	int j;
	double stepSize  = 0.001;
	Vector3d cv; // center-vertex
	Vector3d v_i, v_j, v_k;
	vector<unsigned int> idList;

//	double dc; // distance from pivot point k to center point c
	
	idList.clear();
	for (i=0; i<Triangles.size(); i++)
	{
		int count = 0;

		if(inShadowArea(vBox_min,vBox_max,vertices[Triangles[i].i])) count++;
		if(inShadowArea(vBox_min,vBox_max,vertices[Triangles[i].j])) count++;
		if(inShadowArea(vBox_min,vBox_max,vertices[Triangles[i].k])) count++;
		
		// add triangle to list if one or two vertices lie inside the shadow area
		if(count == 1 || count == 2)
		{
			idList.push_back(i);
		}
	}

	unsigned int id;
	double smoothing = 0.05;
	//while (idList.size() > 0)
	for(j=0;j<100;j++)
	{
		for (i=0; i<idList.size(); i++)
		{
			id= idList[i];
			
			v_i = vertices[Triangles[id].i];
			v_j = vertices[Triangles[id].j];
			v_k = vertices[Triangles[id].k];
			
			if (inShadowArea(vBox_min,vBox_max,vertices[Triangles[id].i]))
				vertices[Triangles[id].i] += (v_j-v_i + v_k-v_i)*smoothing;
		
			if (inShadowArea(vBox_min,vBox_max,vertices[Triangles[id].j]))
				vertices[Triangles[id].j] += (v_k-v_j + v_i-v_j)*smoothing;

			if (inShadowArea(vBox_min,vBox_max,vertices[Triangles[id].k]))
				vertices[Triangles[id].k] += (v_j-v_k + v_i-v_k)*smoothing;

			if (!inShadowArea(vBox_min,vBox_max,vertices[Triangles[id].i]) && 
				!inShadowArea(vBox_min,vBox_max,vertices[Triangles[id].j]) &&
				!inShadowArea(vBox_min,vBox_max,vertices[Triangles[id].k]))
			{
				idList.erase(idList.begin()+i);
				i--;
			}
		}	
	}

	// Create ray and test for intersection with axis aligned bounding box
	for (i=0; i<vertices.size(); i++)
	{
			while (inShadowArea(vBox_min,vBox_max,vertices[i]))
			{	
				cv = vertices[i]-center;
				if (cv.norm() < 2 * stepSize) break; // break loop to avoid mesh self intersections
				cv.normalize();
				vertices[i] -= cv * stepSize;
			}			
	}
}

bool CWorkspaceHull::inShadowArea(Vector3d& vBox_min, Vector3d& vBox_max, Vector3d& vertex)
{
	int j;	
	CBox box;
	Vector3d vb; // Direction vectors:  vertex-base
	double db[2], dbox; // distance from pivot point k to corners of box j
	CRay ray;
	
	box = CBox(vBox_min,vBox_max);

	// Loop over all pivot points
	for (j=0; j < pRobot->getNow();j++)
	{
		vb = pRobot->getBase(j)-vertex;

		//dc= length(center - pRobot->getBase(j)); 
		db[0] = (box.parameters[0] - pRobot->getBase(j)).norm(); 
		db[1] = (box.parameters[1] - pRobot->getBase(j)).norm();

		if (db[0] < db[1]){ dbox = db[0];}else{dbox = db[1];}

		// check if box lies between pivot point and vertex
		if ( dbox < vb.norm()) 
		{
			ray = CRay(vertex,vb); // ray from vertice to pivot point j
			if(box.intersect(ray)) return true;					
		}
	}

	return false;
}

void CWorkspaceHull::makeSpherePatch(unsigned int edgeID0,unsigned int edgeID1,unsigned int edgeID2,int depth)
{
	unsigned int i;
	unsigned int vNewID[3];
	unsigned int edgeID[3];
	unsigned int cEdgeID[3];

	edgeID[0] = edgeID0; edgeID[1] = edgeID1; edgeID[2] = edgeID2; 

	if (depth==0)
    {
		int i = edges[edgeID[0]].v1ID; 
		int j = edges[edgeID[1]].v1ID; 
		int k = edges[edgeID[2]].v1ID; 

		Triangles.push_back(CTriangleIndices(i,j,k));
    }
    else
    {
		depth--;

		for (i=0; i<3; i++)
		{
			if( edges[edgeID[i]].ed1ID == -1)// if edge is not splitted
			{
				// Split edge
				int v1ID = edges[edgeID[i]].v1ID;
				int v2ID = edges[edgeID[i]].v2ID;
				Vector3d vNew =(vertices[v1ID]+vertices[v2ID]); vNew.normalize(); // Compute new vertex
			
				vertices.push_back(vNew); // add new vertex to vertex list
				vNewID[i] = vertices.size() - 1;

				edges.push_back(CEdge(v1ID,vNewID[i],-1,-1)); // add first child edge
				edges[edgeID[i]].ed1ID = edges.size() - 1; // link parent edge to first child edge
				edges.push_back(CEdge(vNewID[i],v2ID,-1,-1)); // add second child edge
				edges[edgeID[i]].ed2ID = edges.size() - 1; // link parent edge to second child edge 
			}
			else
			{
				// if edge is already subdivided, use second vertex from first segment as new vertex
				unsigned int firstSegmentID = edges[edgeID[i]].ed1ID;
				vNewID[i] = edges[firstSegmentID].v2ID;
			}
		}
			
		// Create 3 triangles around the centered triangle
		edges.push_back(CEdge(vNewID[0],vNewID[2],-1,-1)); // create first edge of the centered triangle
		cEdgeID[0] = edges.size() - 1;
		makeSpherePatch(edges[edgeID[0]].ed1ID,cEdgeID[0],edges[edgeID[2]].ed2ID,depth);

		edges.push_back(CEdge(vNewID[1],vNewID[0],-1,-1)); // create second edge of the centered triangle
		cEdgeID[1] = edges.size() - 1;
		makeSpherePatch(edges[edgeID[1]].ed1ID,cEdgeID[1],edges[edgeID[0]].ed2ID,depth);

		edges.push_back(CEdge(vNewID[2],vNewID[1],-1,-1)); // create third edge of the centered triangle
		cEdgeID[2] = edges.size() - 1;
		makeSpherePatch(edges[edgeID[2]].ed1ID,cEdgeID[2],edges[edgeID[1]].ed2ID,depth);
		
		// Create centered triangle
		makeSpherePatch(cEdgeID[0],cEdgeID[1],cEdgeID[2],depth);
	}

	// invert parent edges
	edges[edgeID[0]].invert();
	edges[edgeID[1]].invert();
	edges[edgeID[2]].invert();
}

/*! Create a unit sphere with radius 1 around center. The smoothness of the sphere is controlled 
 *  through the member variable IterationDepth. 
 */
void CWorkspaceHull::makeSphere()
{	
	Triangles.clear();
	vertices.clear();
	edges.clear();

	vertices.push_back(Vector3d( 1, 0, 0)); //  vertex 0
	vertices.push_back(Vector3d( 0, 1, 0)); //	vertex 1
	vertices.push_back(Vector3d( 0, 0, 1));	//	vertex 2
	vertices.push_back(Vector3d(-1, 0, 0));	//	vertex 3
	vertices.push_back(Vector3d( 0,-1, 0));	//	vertex 4
	vertices.push_back(Vector3d( 0, 0,-1)); //	vertex 5
	
	// the edge vectors have to be oriented positive
	// with regards to the triangle in which they are used first
	edges.push_back(CEdge(0,1,-1,-1)); // edge 0
	edges.push_back(CEdge(1,2,-1,-1)); // edge 1
	edges.push_back(CEdge(2,0,-1,-1)); // edge 2

	edges.push_back(CEdge(4,0,-1,-1)); // edge 3
	edges.push_back(CEdge(4,5,-1,-1)); // edge 4
	edges.push_back(CEdge(0,5,-1,-1)); // edge 5
	
	edges.push_back(CEdge(3,4,-1,-1)); // edge 6
	edges.push_back(CEdge(2,4,-1,-1)); // edge 7
	edges.push_back(CEdge(2,3,-1,-1)); // edge 8

	edges.push_back(CEdge(1,3,-1,-1)); // edge 9
	edges.push_back(CEdge(5,1,-1,-1)); // edge 10
	edges.push_back(CEdge(3,5,-1,-1)); // edge 11

	// define triangles by their edges using counter clockwise convention
	// upper part of octahedron
	makeSpherePatch(0,1,2,IterationDepth);
	makeSpherePatch(3,2,7,IterationDepth);	
	makeSpherePatch(6,7,8,IterationDepth);
	makeSpherePatch(9,8,1,IterationDepth);

	//lower part of octahedron
	makeSpherePatch(0,5,10,IterationDepth);
	makeSpherePatch(5,3,4,IterationDepth);
	makeSpherePatch(4,6,11,IterationDepth);		
	makeSpherePatch(11,9,10,IterationDepth);
	
	// move the sphere to center
	for (unsigned int i=0; i<vertices.size(); i++)
		vertices[i]+=center;
	// set the region of interest to the full range
	roiStart=0;
	roiEnd=vertices.size();
}

//! save the workspace to an STL file 
bool CWorkspaceHull::saveWorkspace(const string& filename)
{
	// local data structure to write results into a STL file
	struct CVector3f 
	{
		float x,y,z;
		CVector3f() : x(0),y(0),z(0) {}
		CVector3f(const float& X,const float& Y,const float& Z) : x(X),y(Y),z(Z) {}
	};

	struct CSTLRecord
	{
		CVector3f n,a,b,c;
		char res[2];
	};

	ofstream file(filename.c_str(),ios::binary);
	if (!file.good())
		return false;
	char buf[84] = {};	// header (80) + length
	unsigned int *length = (unsigned int*)(buf+80);
	*length = FinishedTriangles;	// write the number of triangles to the file
	file.write(buf,84);				// write the header
	
	CSTLRecord record;
	for (int i=0; i<FinishedTriangles; i++)
	{
		record.a = CVector3f((float)vertices[Triangles[i].i].x(), (float)vertices[Triangles[i].i].y(), (float)vertices[Triangles[i].i].z());
		record.b = CVector3f((float)vertices[Triangles[i].j].x(), (float)vertices[Triangles[i].j].y(), (float)vertices[Triangles[i].j].z());
		record.c = CVector3f((float)vertices[Triangles[i].k].x(), (float)vertices[Triangles[i].k].y(), (float)vertices[Triangles[i].k].z());
		file.write((char*)(&record.n.x),50);
	}
	return true;
}

/*! save the space occupied by the cables to an STL file
 *  \param filename [in] is the name of the STL file generated by the function
 *  \param cables [in] specifies for which cable the export is done; 
 *         if not given the envelope is generated for all cables
 *  \return true, if successful
 *  \remark The current implementation is rather simple and does not
 *  remove inner objects. An improved version may be able to significantly
 *  reduce the size of the generated file without loss of information.
 */
bool CWorkspaceHull::saveCableSpaceEnvelope(const string& filename, int cables)
{
	// check for parameters
	if (cables>pRobot->getNow())
		return false;
	if (cables<-1)
		return false;

	// local data structure to write results into a STL file
	struct CVector3f 
	{
		float x,y,z;
		CVector3f() : x(0),y(0),z(0) {}
		CVector3f(const float& X,const float& Y,const float& Z) : x(X),y(Y),z(Z) {}
		void convert(const Vector3d& v) { x=(float)v.x(); y=(float)v.y(); z=(float)v.z(); }
	};

	struct CSTLRecord
	{
		CVector3f n,a,b,c;
		char res[2];
	};

	int legsBegin = cables==-1?0:cables;
	int legsEnd = cables==-1?pRobot->getNow():cables+1;

	// open file
	ofstream file(filename.c_str(),ios::binary);
	if (!file.good())
		return false;
	// write the header
	char buf[84] = {};	// header (80) + length
	unsigned int *length = (unsigned int*)(buf+80);
	*length = FinishedTriangles*4*(legsEnd-legsBegin);	// write the number of triangles to the file
	file.write(buf,84);	// write the header
	
	CSTLRecord record;
	record.n.x=0;
	record.n.y=0;
	record.n.z=1;
	// use the first matrix in the orientation set for the platform. Perhaps one day we also loop through all orientations
	Matrix3d R = OrientationSet[0];
	// loop through the respective legs
	for (int j=legsBegin; j<legsEnd; j++)
	{
		// loop through the vertices of the workspace
		for (int i=0; i<FinishedTriangles; i++)
		{
			// compute effective vertices of the tetrahedron
			Vector3d A,BA,BB,BC,N;
			A = pRobot->getBase(j);
			BA = vertices[Triangles[i].i] + R*pRobot->getPlatform(j);
			BB = vertices[Triangles[i].j] + R*pRobot->getPlatform(j);
			BC = vertices[Triangles[i].k] + R*pRobot->getPlatform(j);
			// generate the STL chunk for the four triangles of the tetrahedron and write each into the file
			record.a.convert(A); record.b.convert(BA); record.c.convert(BB);  record.n.convert((BA-A).cross(BB-A));   file.write((char*)(&record.n.x),50);
			record.a.convert(A); record.b.convert(BB); record.c.convert(BC);  record.n.convert((BB-A).cross(BC-A));   file.write((char*)(&record.n.x),50);
			record.a.convert(A); record.b.convert(BC); record.c.convert(BA);  record.n.convert((BC-A).cross(BA-A));   file.write((char*)(&record.n.x),50);
			record.a.convert(BA); record.b.convert(BB); record.c.convert(BC); record.n.convert((BB-BA).cross(BC-BA)); file.write((char*)(&record.n.x),50);
		}
	}
	return true;	
}

/*! Matlab diagram template generator
 *  generate a matlab script file with filename
 *  and write matlab code to plot the current workspace as a
 *  3d matlab plot. matlab diagram are intented to produce paper quality
 *  images of the workspace.
 *  \param [in] filename name of the matlab script. the extension .m is automatically appended
 *  \return true, if successful, otherwise false
 *  \todo Implement some possibilities to configure the matlab plot settings
 */
bool CWorkspaceHull::saveWorkspaceMatlab(const string& filename)
{
	// save in matlab mode
	ofstream mfile((filename+".m").c_str());
	// write all Vertex-Data starting points
	mfile << "DAT=[ ";
	for (int i=0; i<FinishedTriangles; i++)
		mfile<< vertices[Triangles[i].i].x() << " "
			 << vertices[Triangles[i].j].x() << " "
			 << vertices[Triangles[i].k].x() << " "
			 << vertices[Triangles[i].i].y() << " "
			 << vertices[Triangles[i].j].y() << " "
			 << vertices[Triangles[i].k].y() << " "
			 << vertices[Triangles[i].i].z() << " "
			 << vertices[Triangles[i].j].z() << " "
			 << vertices[Triangles[i].k].z() << ";\n";
	mfile << "]';\n";

	// write the plotting command and configure the plot
	mfile<< "axes('FontSize',16)\n"
		<< "patch(DAT(1:3,:),DAT(4:6,:),DAT(7:9,:),1,'LineWidth',1)\n"
		<< "grid on\n"
		<< "box on\n"
		<< "xlabel('x [m]')\n"
		<< "ylabel('y [m]')\n"
		<< "zlabel('z [m]')\n"
		<< "view([35 30])\n"
		<< "print -depsc " << filename << "\n";

	return true;
}

/*! Export of the vertices to csv
 *  \param [in] filename name of the csv-file
 *  \return true, if successful, otherwise false
 */
bool CWorkspaceHull::saveVerticesCSV(const string& filename)
{
	ofstream mfile((filename).c_str());

	unsigned int i;

	//! Loop through the vertices and stream them as CSV
	for ( i = 0; i < vertices.size(); i++ )
	{
		mfile<< vertices[i].x() << "; "
			 << vertices[i].y() << "; "
			 << vertices[i].z() << ";\n";
	}

	return true;
}


bool CWorkspaceHull::getWorkspaceGeometry(vector<Vector3d> &vertices_out, vector<CTriangleIndices> &triangles_out, int &finishedTriangles_out)
{
	vertices_out = vertices;
	triangles_out = Triangles;
	finishedTriangles_out = FinishedTriangles;
	return true;
}


/*! clip the workspace hull with a given axis aligned bounding box. This can 
 *  be understood as applying the logical AND opertation to the sets given by the 
 *  current workspace and the box represented by bbmin,bbmax. 
 *  \param bbmin [in] the "lower" corner of the bounding box used to clip the workspace
 *  \param bbmax [in] the "upper" corner of the boundubg box used to clip the workspace
 *  \return true, if clipping was successful, otherwise false; especially if center is not inside the box
 *  \remark this function will only work correctly, if the center of projection (member "center)
 *  is inside the clipping box; this depends on the convexity (or more precisely star-form) assumption 
 *  on the shape of the workspace. This limitation comes from the data model not from the clipping.
 */
bool CWorkspaceHull::clipByBoundingBox(const Vector3d& bbmin, const Vector3d& bbmax)
{
	// paranoia testing: create the "true" min,max vectors from the given ones
	Vector3d bbMin(min(bbmin.x(),bbmax.x()),
				   min(bbmin.y(),bbmax.y()),
				   min(bbmin.z(),bbmax.z()));
	Vector3d bbMax(max(bbmin.x(),bbmax.x()),
				   max(bbmin.y(),bbmax.y()),
				   max(bbmin.z(),bbmax.z()));

	// check if center is enclosed by the box
	if (center.x()<bbMin.x() || center.x() > bbMax.x() ||
		center.y()<bbMin.y() || center.y() > bbMax.y() ||
		center.z()<bbMin.z() || center.z() > bbMax.z())
		return false;

	for (unsigned int i=0; i<vertices.size(); i++)
	{
		// compute direction vector of the ray from center to the vertex
		Vector3d u=vertices[i]-center;
		double lambda = u.norm();
		u.normalize();
		if (lambda<0) 
			continue;

		// check for x+ plane: normal=(1,0,0); one point on the plane bbMax
		double lambda0 = (bbMax-center).x() / u.x();
		if (lambda0>0 && lambda>lambda0)
			lambda = lambda0;
		// check for y+ plane: normal=(0,1,0); one point on the plane bbMax
		lambda0 = (bbMax-center).y() / u.y();
		if (lambda0>0 && lambda>lambda0)
			lambda = lambda0;		
		// check for z+ plane: normal=(0,0,1); one point on the plane bbMax
		lambda0 = (bbMax-center).z() / u.z();
		if (lambda0>0 && lambda>lambda0)
			lambda = lambda0;

		// check for x- plane: normal=(-1,0,0); one point on the plane bbMin
		lambda0 = (bbMin-center).x() / u.x();
		if (lambda0>0 && lambda>lambda0)
			lambda = lambda0;
		// check for y- plane: normal=(0,-1,0); one point on the plane bbMin
		lambda0 = (bbMin-center).y() / u.y();
		if (lambda0>0 && lambda>lambda0)
			lambda = lambda0;
		// check for z- plane: normal=(0,0,-1); one point on the plane bbMin
		lambda0 = (bbMin-center).z() / u.z();
		if (lambda0>0 && lambda>lambda0)
			lambda = lambda0;

		// recompute the vertex
		vertices[i] = center + lambda * u;
	}
	this->calculateWorkspaceProperties();
	return true;
}

/*! This function determines an approximation of a conic hull for the cable span, 
 *  i.e. the the space that is occupied by the cables.
 *  The cable span is stored in the member variable CableSpanMatrix for later use. 
 *  The matrix has the size (number of segments) times (number of wires) and stores 
 *  the angle between the central axis of the cable span and its surface.
 *  Secondly, the content of the matrix CableSpanTriangulation is determined which
 *  has a dimension of 3 times (number of segments * number of wires). This matrix 
 *  contains NoW polygons with the cross-section of the cable span. Connecting these
 *  vertices of the polygon with the respective proximal anchor point a_i results 
 *  provides a triangulation of the i-th cable span
 * 
 *  \remark The matrix CableSpanTriangulation is not yet checked to provide correct 
 *  results
 * 
 *  \return true, if the computation was successfully completed, otherwise false.
 */
bool CWorkspaceHull::calculateCableSpan()
{
	// the following variables are subject to get protected class members
	const int segments=36;	// the number of samples used within the 

	// assign the respective dimension to the data model
	CableSpanMatrix.resize(segments, pRobot->getNow());
	CableSpanTriangulation.resize(3, segments * pRobot->getNow());

	if (vertices.size()<1)
		return false;
	// start the determination of calculation time
	time_t start = clock();
	// for each cable
	for (int i=0; i<pRobot->getNow(); i++)
	{
		// estimate the central ray a_i -> center
		Vector3d uc = center - pRobot->getBase(i);
		uc.normalize();
		
		// to compute the vector phi, we have to define a local frame. we perform polar 
		// sorting to simplify the conic hull of the cable span vectors
		// firstly, we construct a local coordinate frame at anchor points A_i with its
		// z-axis pointing towards the project center. the x axis is
		// given by the first vertex, the z-axis is given by uc, the y-axis is just 
		// orthogonal to the x- and z- axis
		Vector3d e1 = vertices[0]-center;
		e1-=e1.dot(uc)*uc;
		e1.normalize();
		Vector3d e2 = uc.cross(e1);
		e2.normalize();

		// the data model for intermediate storage of the span (mapping phi -> cos_alpha)
		map<double,double> span;
		// perform a coordinate transformation from world frame to polar form w.r.t. a_i and the axis uc
		for (unsigned int j=0; j<vertices.size(); j++)
		{
			// compute the span vector
			Vector3d ui = vertices[j]+pRobot->getPlatform(i)-pRobot->getBase(i);
			ui.normalize();
			// compute its angles alpha (or more precisely its cosine) and phi w.r.t. to e1,e2,uc
			double cos_alpha=ui.dot(uc);
			double phi=atan2(ui.dot(e2),ui.dot(e1));
			// store result in local storage span (since it is a map we implicitly sort the data thereby)
			span[phi]=cos_alpha;
		}
		
		// now we try to reduce the number of points by deleting intermediat points if
		// a) they smaller than their neighbours and
		// b) the distance between the neighbours is smaller that 2*PI/36
		
		const double delta_phi = 2*MO_PI/segments;
		for (int segment=0; segment<segments; segment++)
		{
			double low = -MO_PI+segment*delta_phi;
			double high = -MO_PI+(segment+1)*delta_phi;
			map<double,double>::iterator itor_low = span.lower_bound(low);
			map<double,double>::iterator itor_high = span.upper_bound(high);
			// \todo WARNING: We might do other comparison because we saved cos_alpha (not alpha itself)
			double max_alpha=1;
			for (; itor_low!=itor_high; itor_low++)
			{
				max_alpha = min(max_alpha,itor_low->second);
			}
			// at this point we store the data in a matrix and do the backward transformation by storing the
			// actual angle (until now we have stored the cosine of the angle)
			CableSpanMatrix(segment, i) = acos(max_alpha);
//			cout << i << " | " << low << ": " <<  max_alpha << endl;
			// compute the projection of the determined direction on the center plain
			Matrix3d R; R << e1,e2,uc;
			CableSpanTriangulation.block(0, segments*i + segment,3 ,1) =
				pRobot->getBase(i) + R * (center-pRobot->getBase(i)).norm()*Vector3d(cos((low+high)/2)*tan(acos(max_alpha)),sin((low+high)/2)*tan(acos(max_alpha)),1.0);
		}
		
		// Only for debug purpose: print the resulting table; we will later consider something to store the results
		// cout << "Table for winch " << i << endl;
		// cout << "*****************\n";
		// for (map<double,double>::iterator itor=span.begin(); itor!=span.end(); itor++)
		//		cout << itor->first << ", " << itor->second << endl;
	}

	// we store the latest computation time in the state variable
	calculationTime=(int)(clock()-start);

//	cout << "CableSpanMatrix" << CableSpanMatrix << endl;
//	cout << "CableSpanTriangulation" << CableSpanTriangulation << endl;

	return true;
}



//////////////////////////////////////////////////////////////////////////////
// CWorkspaceGrid
//////////////////////////////////////////////////////////////////////////////

CWorkspaceGrid::CWorkspaceGrid(CRobotData& robot)
	: CWorkspaceAlgorithm(robot)
{
	bCheckInternalPoints=true; // for inner box 
	bbmin=Vector3d::Zero(); // set the edges of the inner bounding box to 0
	bbmin=Vector3d::Zero();

}

CWorkspaceGrid::~CWorkspaceGrid()
{
}

/*! create a regular grid depending on the current motion pattern of the robot.
 *  The grid is either 2D or 3D depending on the motion pattern.
 *  \param [in] min The lower corner of the regular grid.
 *  \param [in] max The upper corner of the regular grid.
 *  \param [in] eps The step size in x,y,z, respectively. If eps is omitted
 *  eps is automatically determined such that the grid has 10 or 11 points in
 *  each direction. Round-off errors might cancle out the last grid point
 *  to reach the values given in max. Be careful to choose eps resonablely
 *  large, because small values might lead to excessive usage of memory and 
 *  time.
 *  \return true, if the grid was successfully generated, otherwise false
 */
bool CWorkspaceGrid::makeGrid(const Vector3d& min, const Vector3d& max, const Vector3d& eps)
{
	const double eps_adjust=0.0001; // paramater to avoid rounding error

	// check the parameter
	if (max.x()<min.x() || max.y()<min.y() || max.z()<min.z())
		return false;
	if (eps.x()<0 || eps.y()<0 || eps.z()<0)
		return false;
	
	Vector3d Eps;
	if (eps.x()>0 && eps.y()>0 && eps.z()>0)
	{
		Eps = eps;
		// auto adjust the accuracy of the discretisation
		Eps.x()=(max.x()-min.x())/(ceil((max.x()-min.x())/Eps.x()))-eps_adjust;
		Eps.y()=(max.y()-min.y())/(ceil((max.y()-min.y())/Eps.y()))-eps_adjust;
		Eps.z()=(max.z()-min.z())/(ceil((max.z()-min.z())/Eps.z()))-eps_adjust;
	}
	else
		Eps = (max-min)/10.0;	// calculate Eps for 10 or 11 steps
	
	// delete all data in the current data model
	vertices.clear();

	if (pRobot->getMotionPattern()==CRobotData::MP1T)
	{ 
		Vector3d pos = 0.5*(min+max);
		for (pos.x() = min.x(); pos.x()<=max.x(); pos.x()+=Eps.x())
			vertices.push_back(pos);
	}
	else if (pRobot->getMotionPattern()==CRobotData::MP2T || pRobot->getMotionPattern()==CRobotData::MP1R2T )
	{
		// create a regular 2D grid in x,y direction. z is the middle between min and max
		Vector3d pos;
		pos.z()=0.5*(min.z()+max.z());
		for (pos.x() = min.x(); pos.x()<=max.x(); pos.x()+=Eps.x())
			for (pos.y() = min.y(); pos.y()<=max.y(); pos.y()+=Eps.y())
				vertices.push_back(pos);
	}
	else if (pRobot->getMotionPattern()==CRobotData::MP3R3T || pRobot->getMotionPattern()==CRobotData::MP2R3T || pRobot->getMotionPattern()==CRobotData::MP3T)
	{
		/*
		// verify a regular grid given by min,max,Eps
		Vector3d pos;
		for (pos.x() = min.x(); pos.x()<=max.x(); pos.x()+=Eps.x())
			for (pos.y() = min.y(); pos.y()<=max.y(); pos.y()+=Eps.y())
				for (pos.z() = min.z(); pos.z()<=max.z(); pos.z()+=Eps.z())
					vertices.push_back(pos);
		*/
		// new implementation which prepares data for the determination of the biggest inner box
		// number of discrete elements in x,y and z direction
		i_max=0;
		j_max=0;
		k_max=0;
		// verify a regular grid given by min,max,Eps
		Vector3d pos;
		for (pos.x() = min.x(); pos.x()<=max.x(); pos.x()+=Eps.x())
		{
			i_max++;
			for (pos.y() = min.y(); pos.y()<=max.y(); pos.y()+=Eps.y())
			{
				j_max++;
				for (pos.z() = min.z(); pos.z()<=max.z(); pos.z()+=Eps.z())
				{
					k_max++;
					vertices.push_back(pos);
				}
			}
		}
		
		k_max=k_max/j_max;
		j_max=j_max/i_max;
	}

	return true;
}

//! return a random number in the range between lower and upper
double CWorkspaceGrid::randomNumber(const double& lower, const double& upper)
{
	return ((double)rand()/(double)RAND_MAX) * (upper-lower) + lower;
}

//! create a randomly distributed grid within the box (min/max) with size positions
//! depending on the motion pattern of the current robot the functions creates a linear
//! planar or spatial distribution of positions.
//! \return true, if successful
bool CWorkspaceGrid::makeRandomGrid(const Vector3d& min, const Vector3d& max, const unsigned int size)
{
	// check the parameter
	if (max.x()<min.x() || max.y()<min.y() || max.z()<min.z())
		return false;

	// delete all data in the current data model
	vertices.clear();

	if (pRobot->getMotionPattern()==CRobotData::MP1T)
	{ 
		for (unsigned int i=0; i<size; i++)
			vertices.push_back(Vector3d(randomNumber(min.x(), max.x()),0,0));
	}
	else if (pRobot->getMotionPattern()==CRobotData::MP2T || pRobot->getMotionPattern()==CRobotData::MP1R2T )
	{
		for (unsigned int i=0; i<size; i++)
			vertices.push_back(Vector3d(randomNumber(min.x(), max.x()),randomNumber(min.y(), max.y()),0));
	}
	else if (pRobot->getMotionPattern()==CRobotData::MP3R3T || pRobot->getMotionPattern()==CRobotData::MP2R3T || pRobot->getMotionPattern()==CRobotData::MP3T)
	{
		for (unsigned int i=0; i<size; i++)
			vertices.push_back(Vector3d(randomNumber(min.x(), max.x()),randomNumber(min.y(), max.y()),randomNumber(min.z(), max.z())));
	}

	return true;
}

//! create a positions within the box min/max with Gaussian normal distributions
bool makeGaussianGrid(const Vector3d& min, const Vector3d& max, const Vector3d& sigma, const unsigned int size)
{ return false; }

//! create a grid with concentric cyclinders 
bool CWorkspaceGrid::makeCylinderGrid(const Vector3d& min, const Vector3d& max, char normal, int layerSteps, int radialSteps, int angularSteps)
{
	// check the parameter
	if (normal!='x' && normal!='y' && normal!='z')
		return false;
	if (max.x()<min.x() || max.y()<min.y() || max.z()<min.z())
		return false;
	if (layerSteps<=0 || radialSteps<=0 || angularSteps<=0)
		return false;

	// delete all data in the current data model
	vertices.clear();

	Vector3d center = (max+min)/2.0;
	Vector3d radii = (max-min)/2.0;

	// deal with the z-axis case
	if (normal=='z')
	{
		for (int layer=0; layer<=layerSteps; layer++)
			for (int radius=0; radius<=radialSteps; radius++)
				for (int angle=0; angle<=angularSteps; angle++)
				{
					vertices.push_back(Vector3d(
						center.x() + radii.x() * radius * sin(angle*MO_PI*2/angularSteps)/radialSteps,
						center.y() + radii.y() * radius * cos(angle*MO_PI*2/angularSteps)/radialSteps,
						min.z() + layer * ( max.z() - min.z() ) / layerSteps
					));
				}
		return true;
	}
	else // till now, we do not deal with the cases for x and y direction
		return false;
}

//! check all positions in the grid structure and store all positions with positive
//! result in workspace and all others in workspace_out. 
//! This function overwrites computation time.
//! \return true, if the workspace computation was successful (i.e. without errors). 
bool CWorkspaceGrid::calculateWorkspace()
{
	if (vertices.size()<=0)
		return false;

	// clear the data model before computation is started
	workspace.clear();
	workspace_out.clear();

	vertices_expanded.clear();
	CPointFeasible PointFeasible;

	// start time measurement
	time_t start=clock();
	for (unsigned int i=0; i<vertices.size(); i++)
		if (workspaceEvaluator(vertices[i]))
		{
			workspace.push_back(vertices[i]);
			PointFeasible.point=vertices[i];
			PointFeasible.bFeasible=true;
			vertices_expanded.push_back(PointFeasible);
		}
		else
		{
			workspace_out.push_back(vertices[i]);
			PointFeasible.point=vertices[i];
			PointFeasible.bFeasible=false;
			vertices_expanded.push_back(PointFeasible);
		}

	// stop time measurement
	calculationTime = (int)(clock()-start);
	return true;
}

/*! calculate the orientation workspace by sampling through the orientations in orientationSet
 * \param r [in] the position where the orientation workspace is computed
 * \param coverage [out] the percentage of the orienations belonging to the workspace
 * \return true, if successful
 */
bool CWorkspaceGrid::calculateOrientationWorkspace(const Vector3d& r, double& coverage)
{
	if (OrientationSet.size()<=0)
		return false;
	// start time measurement
	time_t start=clock();
	
	int OrientationCount = 0;
	// check all orientations in the orienation set
	for (unsigned int i=0; i<OrientationSet.size(); i++)
	{
		if (testPose(r, OrientationSet[i]))
			OrientationCount++;
	}
	coverage = (double)OrientationCount / (double)OrientationSet.size();
	// stop time measurement
	calculationTime = (int)(clock()-start);
	return true;
}

double CWorkspaceGrid::getOrienationWorkspaceCoverage(const Vector3d& r)
{
	double coverage=0.0;
	calculateOrientationWorkspace(r,coverage);
	return coverage;
}

/*! Check if the axis aligned box given through (min, max) is fully inside the 
 *  workspace. verifyWorkspaceBox generates a new grid overwriting an existing
 *  grid. 
 *  \param min [in] the lower limits of the cartesian coordinates of the box 
		to be verified
 *  \param max [in] the upper limits of the box to be verified
 *  \param eps [in] the stepsize for the discritisation; the parameter is 
		optional; if no values (or a zero vector) is set, the stepsize is 
		calculate to perform 10 evaluation per dimension
 *  \return true, if the algorithms succeeds in verifying the workspace, 
		otherwise false
 */
bool CWorkspaceGrid::verifyWorkspaceBox(const Vector3d& min, const Vector3d& max, const Vector3d& eps)
{
	if (!makeGrid(min,max,eps))
		return false;

	for (unsigned int i=0; i<vertices.size(); i++)
		if (!workspaceEvaluator(vertices[i]))
			return false;
	return true;
}


//! determine the box with the biggest volume which lies inside the workspace
bool CWorkspaceGrid::calculateInnerBox()
{
	time_t start=clock();

	if (vertices_expanded.size()<=0)
	{
		cout << "workspace list is empty, call the evaluateGrid first\n";
		bbmin=Vector3d::Zero(); // set the edges of the inner bounding box to 0
		bbmin=Vector3d::Zero();
		return false;
	}

	double v_opt=0; // optimal volume [m^3]
	int i_min_opt, j_min_opt, k_min_opt, i_max_opt, j_max_opt, k_max_opt; // elements describing the two edges of the inner box (i=x, j=y, k=z)

	// maybe add here a determination of the outer bounding box to minimize the scope for the following loops

	for (int I_max=0; I_max<i_max; I_max++)
		for (int I_min=0; I_min<I_max; I_min++)
			for (int J_max=0; J_max<j_max; J_max++)
				for (int J_min=0; J_min<J_max; J_min++)
					for (int K_max=0; K_max<k_max; K_max++)
						for (int K_min=0; K_min<K_max; K_min++)
						{
							// check all 8 edges of the box
							int I, J, K;
							int iSum=0;
							I=I_min; J=J_min; K=K_min;
							iSum+=vertices_expanded[K+J*k_max+I*k_max*j_max].bFeasible;
							I=I_min; J=J_min; K=K_max;
							iSum+=vertices_expanded[K+J*k_max+I*k_max*j_max].bFeasible;
							I=I_min; J=J_max; K=K_min;
							iSum+=vertices_expanded[K+J*k_max+I*k_max*j_max].bFeasible;
							I=I_min; J=J_max; K=K_max;
							iSum+=vertices_expanded[K+J*k_max+I*k_max*j_max].bFeasible;
							I=I_max; J=J_min; K=K_min;
							iSum+=vertices_expanded[K+J*k_max+I*k_max*j_max].bFeasible;
							I=I_max; J=J_min; K=K_max;
							iSum+=vertices_expanded[K+J*k_max+I*k_max*j_max].bFeasible;
							I=I_max; J=J_max; K=K_min;
							iSum+=vertices_expanded[K+J*k_max+I*k_max*j_max].bFeasible;
							I=I_max; J=J_max; K=K_max;
							iSum+=vertices_expanded[K+J*k_max+I*k_max*j_max].bFeasible;

							if (iSum ==8) // box lies in the workspace, if all 8 edges are in the workspace // Idea: add additional check of points inside the box
							{
								
								bbmin=vertices_expanded[K_min+J_min*k_max+I_min*k_max*j_max].point; // edge 1
								bbmax=vertices_expanded[K_max+J_max*k_max+I_max*k_max*j_max].point;  // edge 2
								double v=abs(bbmin.x()-bbmax.x())*abs(bbmin.y()-bbmax.y())*abs(bbmin.z()-bbmax.z()); // calculation of the volume
								if (v>v_opt) // check if the box is better than best known
								{
									if (bCheckInternalPoints) // Idea: check of points inside the box
									{
										for (int i=I_min; i<=I_max; i++)
											for (int j=J_min; j<=J_max; j++)
													for (int k=K_min; k<=K_max; k++)
														if (vertices_expanded[k+j*k_max+i*k_max*j_max].bFeasible==0)
															goto MoveOn; // there is at least one point which is not in the workspace
									}
									v_opt=v;
									i_min_opt=I_min;
									j_min_opt=J_min;									
									k_min_opt=K_min;
									i_max_opt=I_max;
									j_max_opt=J_max;									
									k_max_opt=K_max;
								}
							}
							MoveOn: // label used when not all internal points belong to the workspace
							;
						}

	if ((k_min_opt+j_min_opt*k_max+i_min_opt*k_max*j_max) >= 0 && 
		(k_min_opt+j_min_opt*k_max+i_min_opt*k_max*j_max) < (int)vertices_expanded.size()&&
		(k_max_opt+j_max_opt*k_max+i_max_opt*k_max*j_max) >= 0 && 
		(k_max_opt+j_max_opt*k_max+i_max_opt*k_max*j_max) < (int)vertices_expanded.size())
	{
		// optimal inner box is described by bbmin and bbmax:
		bbmin=vertices_expanded[k_min_opt+j_min_opt*k_max+i_min_opt*k_max*j_max].point;
		bbmax=vertices_expanded[k_max_opt+j_max_opt*k_max+i_max_opt*k_max*j_max].point;

		cout << "resulting box min: " << bbmin.x() << " "<< bbmin.y() << " "<< bbmin.z() <<"\n";
		cout << "resulting box max: " << bbmax.x() << " "<< bbmax.y()<< " " << bbmax.z() <<"\n";
		cout << "resulting box abs: " << bbmax.x()-bbmin.x() << " "<< bbmax.y()- bbmin.y()<< " " << bbmax.z() -bbmin.z() <<"\n";
		cout << "Volume: " << v_opt <<"\n"; 

	}
	else
	{
		cout << "No inner box found: workspace seems to be empty\n";
		bbmin=Vector3d::Zero(); // set the edges of the inner bounding box to 0
		bbmin=Vector3d::Zero();
	}

	calculationTime = (int)(clock()-start);

	cout << "Time for inner box [ms]:" << calculationTime <<"\n";

	return true;
}


/*! Calculate the maximum and minimum cable forces for a given box. The current
 *  setting in the vertex grid are overwritten with the box grid specified.
 *  \param Fmin [out] an array of size number of wires with the minimal wire 
		force for each winch
 *  \param Fmax [out] an array of size number of wires with the maximum wire 
		force for each winch
 *  \param Min [in] the lower bounds on the (x,y,z) coordinates of box
 *  \param Max [in] the upper bound on the (x,y,z) coordinates of box
 *  \param R [in] the orientation of the platform used for the calculation
 *  \param eps [in] the stepsize for the discretisation; the parameter is 
		optional; if no values (or a zero vector) is set, the stepsize is 
		calculated to perform 10 evalutation per dimension
 *  \return true, if successful, otherwise false. In the latter case the 
		results in Fmin and Fmax are undefined
 */
bool CWorkspaceGrid::wireForcesWorkspaceBox(MatrixXd& fmin, MatrixXd& fmax, 
	const Vector3d& Min, const Vector3d& Max, const Matrix3d& R, const Vector3d& eps)
{
	if (!makeGrid(Min,Max,eps))
		return false;
	
	//! this function can only be used if the CPoseProperty* used as
	//! force evaluator is kind-of class CForceDistribution
	CForceDistribution* pFD = dynamic_cast<CForceDistribution*>(pForceDistribution);

	// prepare the force calculation
	MatrixXd w = MatrixXd::Zero(pRobot->getDof(),1);
	MatrixXd F = MatrixXd::Zero(pRobot->getNow(),1);
	MatrixXd f_min = MatrixXd::Constant(pRobot->getNow(),1,pRobot->fmin);
	MatrixXd f_max = MatrixXd::Constant(pRobot->getNow(),1,pRobot->fmax);

	//! implement for all Motion Patterns
	//! \todo: convention for which axis is relevant for which motion pattern!
	//! the following definition might be better placed in a utility function
	if(pRobot->getMotionPattern()==CRobotData::MP1T)
	{
		w(0)=pFD->f.x();
	}
	else if (pRobot->getMotionPattern()==CRobotData::MP2T)// apply the load (only for the dof=2 case)
	{
		w(0)=pFD->f.x();
		w(1)=pFD->f.y();
	}
	else if (pRobot->getMotionPattern()==CRobotData::MP1R2T)
	{
		w(0)=pFD->f.x();
		w(1)=pFD->f.y();
		w(2)=pFD->tau.x();
	}
	else if (pRobot->getMotionPattern()==CRobotData::MP3T)
	{
		w(0)=pFD->f.x();
		w(1)=pFD->f.y();
		w(2)=pFD->f.z();
	}
	else if (pRobot->getMotionPattern()==CRobotData::MP2R3T)
	{
		w(0)=pFD->f.x();
		w(1)=pFD->f.y(); 
		w(2)=pFD->f.z();
		w(3)=pFD->tau.x();
		w(4)=pFD->tau.y();
	}
	else if (pRobot->getMotionPattern()==CRobotData::MP3R3T)
	{	
		// apply the load (only for the dof=6 case)
		w(0)=pFD->f.x();
		w(1)=pFD->f.y(); 
		w(2)=pFD->f.z();
		w(3)=pFD->tau.x();
		w(4)=pFD->tau.y();
		w(5)=pFD->tau.z();
	}
	
	// compute a test evaluation of the structure matrix at the first vertex in the grid
	if (!pFD->getStructureMatrix(vertices[0],R))
	{
		printf("ERROR: first getStructureMatrix failed\n");
		return false;
	}

	if (!pFD->getDistribution(w,f_min,f_max,F))
	{
		printf("ERROR: first getDistribution failed for (%f,%f,%f) \n",Min.x(),Min.y(),Min.z());
		return false;
	}
	// save the first force distribution as refernce for the following evaluations
	fmin = F;
	fmax = F;

	// loop through the remaining vertices in the grid and repeat the evaluation
	for (unsigned int i=1; i<vertices.size(); i++)
	{
		if (!pFD->getStructureMatrix(vertices[i],R))
		{
			printf("ERROR: loop getStructureMatrix failed for (%f,%f,%f)\n",vertices[i].x(),vertices[i].y(),vertices[i].z());
			return false;
		}
		if (!pFD->getDistribution(w,f_min,f_max,F))
		{
			printf("ERROR: loop getDistribtion failed for (%f,%f,%f)\n",vertices[i].x(),vertices[i].y(),vertices[i].z());
			return false;
		}
		for (int i=0; i<pRobot->getNow(); i++)
		{
			if (fmin(i)>F(i))
				fmin(i)=F(i);
			if (fmax(i)<F(i))
				fmax(i)=F(i);
		}
	}
	return true;
}

/*! Save Workspace Grid as text file
 * at this time the code does not follow a particular convention
 * but is simply a list of points separated by commas emulating a csv but
 * vls 20.11.2014
*/
bool CWorkspaceGrid::saveWSGrid(const string& filename)
{
	ofstream file(filename.c_str());
	string separator=" , ";
	//header
	file << "WS.Feasible.x\tWS.Feasible.y\tWS.Feasible.z\tWS.notFeasible.x\tWS.notFeasible.y\tWS.notFeasible.z\n";
	//cout<<workspace.size()<<" v. "<<workspace_out.size();
	for(unsigned int i=0; i<max(workspace.size(),workspace_out.size());i++)
	{	
		if(i>=min(workspace.size(),workspace_out.size()))
		{
			if(workspace.size()<=workspace_out.size())
			{
				file<<"NaN"<<separator<<"NaN"<<separator<<"NaN"<<separator<<workspace_out.at(i).x()<<separator<<workspace_out.at(i).y()<<separator<<workspace_out.at(i).z()<<"\n";
			}
			else
				file<<workspace.at(i).x()<<separator<<workspace.at(i).y()<<separator<<workspace.at(i).z()<<separator<<"NaN"<<separator<<"NaN"<<separator<<"NaN"<<"\n";
		}
		else
			file<<workspace.at(i).x()<<separator<<workspace.at(i).y()<<separator<<workspace.at(i).z()<<separator<<workspace_out.at(i).x()<<separator<<workspace_out.at(i).y()<<separator<<workspace_out.at(i).z()<<"\n";

	}
	return true;
}

/*! Simplified interface to wireForcesWorkspaceBox assuming no rotation 
 *  The calculated forces are stored in local variables and printed after calculation.
 *  \param Min [in] the lower bounds on the (x,y,z) coordinates of box
 *  \param Max [in] the upper bound on the (x,y,z) coordinates of box
 *  \param eps [in] see wireForcesWorkspaceBox
 *  \return true, if successful, otherwise false. 
*/
bool CWorkspaceGrid::wireForcesWorkspaceBoxDriver(const Vector3d& Min, const Vector3d& Max, const Vector3d& eps)
{
	MatrixXd Fmin(pRobot->getNow(),1);
	MatrixXd Fmax(pRobot->getNow(),1);
	Fmin.setZero();
	Fmax.setZero();
	Matrix3d R=Matrix3d::ZRotationMatrix3d(0);
	bool erg = wireForcesWorkspaceBox(Fmin,Fmax,Min,Max,R,eps);
	if(erg)
	{
	cout <<"Min -- Max Forces in each wire (note: No Rotation Tested)"<<endl;
	for (int i=0; i<pRobot->getNow(); i++)
		cout << i << ": " << Fmin(i,0) << " -- " << Fmax(i,0) << endl;
	}
	else
		cout<<"Error When Calculating WorkspaceBox - check for correct Application min max values | not all MotionPatterns fully tested"<<endl;
	
	return erg;
}

//////////////////////////////////////////////////////////////////////////////
// CWorkspaceDifferentialHull
//////////////////////////////////////////////////////////////////////////////

CWorkspaceDifferentialHull::CWorkspaceDifferentialHull(CRobotData& robot)
: CWorkspaceHull(robot)
{
	epsGeometry = 1e-3;
	nHulls = 6*pRobot->getNow();
	// reserve the memory for the differential hulls
	pDiffHull = new CWorkspaceHull*[nHulls];
	for (int i=0; i<nHulls; i++)
	{
		pDiffHull[i] = new CWorkspaceHull(robot);
		// we add the evaluator objects of our parent object to be used by the child
		// hulls; however, sharing these instances may be a problem for parallel
		// processing
		pDiffHull[i]->attachEvaluator(*pForceDistribution,*pKinematics,*pWrenchSet,*pVelocitySet,*pStiffness);
	}
}

CWorkspaceDifferentialHull::~CWorkspaceDifferentialHull()
{
	// free the memory reserved for the hulls
	if (pDiffHull) 
	{
		for (int i=0; i<nHulls; i++)
			delete pDiffHull[i];
		delete [] pDiffHull;
	}
}

/*! Calculate all the child hulls with variated parameters.
 *  The sequance of the variated parameters delta_g (and thus the meaning of the
 *  values in the pDiffHull objects) is as follows
 *  [ A1.x(), A1.y(), A1.z(),  B1.x(),  B1.y(),  B1.z()
 *    ...
 *    Am.x(), Am.y(), Am.z(),  Bm.x(),  Bm.y(),  Bm.z() ]
 *  future implementation may expand this list with additional properties
 *  such as applied forces and torque (f,tau). 
 */
bool CWorkspaceDifferentialHull::calculateWorkspaceDiff()
{
	// 1. calculate the nominal hull
	calculateWorkspace();

	for (int i=0; i<nHulls; i++)
	{
		//	a) 2. copy the actual algorithm paramters (from CWorkspaceHull) to all child hull objects
		pDiffHull[i]->getAlgorithmParameter(*this);
		//	b) reset the i-th hull objects
		//	c) change the i-th input parameters in the robot's geometry structure
		int offset = i%6;
		int cable = i / 6;
		Vector3d old_a,old_b;
		pRobot->getLeg(cable,old_a,old_b);
		switch ( offset )
		{
		case 0:	pRobot->setLeg(cable,old_a+Vector3d(epsGeometry,0,0),old_b); break;
		case 1:	pRobot->setLeg(cable,old_a+Vector3d(0,epsGeometry,0),old_b); break;
		case 2:	pRobot->setLeg(cable,old_a+Vector3d(0,0,epsGeometry),old_b); break;
		case 3:	pRobot->setLeg(cable,old_a,old_b+Vector3d(epsGeometry,0,0)); break;
		case 4:	pRobot->setLeg(cable,old_a,old_b+Vector3d(0,epsGeometry,0)); break;
		case 5:	pRobot->setLeg(cable,old_a,old_b+Vector3d(0,0,epsGeometry)); break;
		}
		// attach the evaluator objects to the current instance; 
		// THIS IS ONLY A HOTFIX; These objects shall be attach in the configuration stage
		pDiffHull[i]->attachEvaluator(*pForceDistribution,*pKinematics,*pWrenchSet,*pVelocitySet,*pStiffness);
		//	d) calculate the i-th hull
		pDiffHull[i]->calculateWorkspace();
		//	e) calculate the properties of the i-th hull
		pDiffHull[i]->calculateWorkspaceProperties();
		//	f) change the i-th input parameter back to the original value
		pRobot->setLeg(cable,old_a,old_b);
	}
	return true;
}

//! collect the results in a matrix
//! the functions does essentially the same 
bool CWorkspaceDifferentialHull::calculateParameterDifferentials()
{
	// nothing to return?
	if (0 >= nHulls)
		return false;
	// resize the result matrix accordingly
	parameterDifferentials.resize(nHulls+1,11);
	// copy the nominal parameters into the first line
	parameterDifferentials.block(0,0,1,11)
		<< Surface,
			Volume,
			CoI.x(),
			CoI.y(),
			CoI.z(),
			bbMin.x(),
			bbMax.x(),
			bbMin.y(),
			bbMax.y(),
			bbMin.z(),
			bbMax.z();
	// fill the remaining rows with the differentials
	for (int i=0; i<nHulls; i++)
		parameterDifferentials.block(i+1,0,1,11) <<
			(pDiffHull[i]->Surface - Surface) / epsGeometry,
			(pDiffHull[i]->Volume - Volume) / epsGeometry,
			(pDiffHull[i]->CoI.x() - CoI.x()) / epsGeometry,
			(pDiffHull[i]->CoI.y() - CoI.y()) / epsGeometry,
			(pDiffHull[i]->CoI.z() - CoI.z()) / epsGeometry,
			(pDiffHull[i]->bbMin.x() - bbMin.x()) / epsGeometry,
			(pDiffHull[i]->bbMax.x() - bbMax.x()) / epsGeometry,
			(pDiffHull[i]->bbMin.y() - bbMin.y()) / epsGeometry,
			(pDiffHull[i]->bbMax.y() - bbMax.y()) / epsGeometry,
			(pDiffHull[i]->bbMin.z() - bbMin.z()) / epsGeometry,
			(pDiffHull[i]->bbMax.z() - bbMax.z()) / epsGeometry;
	return true;
}

//! return the result matrix
MatrixXd CWorkspaceDifferentialHull::getParameterDifferentials() const
{ return parameterDifferentials; }

/*! test version; not all parameter can be printed here ... */
void CWorkspaceDifferentialHull::printDifferential()
{
	calculateParameterDifferentials();
	static const Eigen::IOFormat IODiff(Eigen::StreamPrecision, 0, " | ", "\n", "", "", "", "");
	cout << " i : Surface | Volume | CoG.x() | CoG.x() | CoG.x() | BB.xmin | BB.xmax | BB.ymin | BB.ymax | BB.zmin | BB.zmax \n";
	cout << "num: " << parameterDifferentials.block(0,0,1,11).format(IODiff) << endl;
	for (int i=0; i<nHulls; i++)
		cout << i << " : " << parameterDifferentials.block(i+1,0,1,11).format(IODiff) << endl;

/*	printf(" i : Surface | Volume | CoG.x() | CoG.x() | CoG.x() | BB.xmin | BB.xmax | BB.ymin | BB.ymax | BB.zmin | BB.zmax \n");  
	printf("nom: %4.3f | %4.3f | %4.3f | %4.3f | %4.3f | %4.3f | %4.3f | %4.3f | %4.3f | %4.3f | %4.3f \n", 
			Surface,
			Volume,
			CoI.x(),
			CoI.y(),
			CoI.z(),
			bbMin.x(),
			bbMax.x(),
			bbMin.y(),
			bbMax.y(),
			bbMin.z(),
			bbMax.z());
	for (int i=0; i<nHulls; i++)
	{
		printf("%2i : %4.3f | %4.3f | %4.3f | %4.3f | %4.3f | %4.3f | %4.3f | %4.3f | %4.3f | %4.3f | %4.3f \n",
			i, 
			(pDiffHull[i]->Surface - Surface) / epsGeometry,
			(pDiffHull[i]->Volume - Volume) / epsGeometry,
			(pDiffHull[i]->CoI.x() - CoI.x()) / epsGeometry,
			(pDiffHull[i]->CoI.y() - CoI.y()) / epsGeometry,
			(pDiffHull[i]->CoI.z() - CoI.z()) / epsGeometry,
			(pDiffHull[i]->bbMin.x() - bbMin.x()) / epsGeometry,
			(pDiffHull[i]->bbMax.x() - bbMax.x()) / epsGeometry,
			(pDiffHull[i]->bbMin.y() - bbMin.y()) / epsGeometry,
			(pDiffHull[i]->bbMax.y() - bbMax.y()) / epsGeometry,
			(pDiffHull[i]->bbMin.z() - bbMin.z()) / epsGeometry,
			(pDiffHull[i]->bbMax.z() - bbMax.z()) / epsGeometry);
	}*/
}

CWorkspaceHull* CWorkspaceDifferentialHull::getDifferntialHull(int i)
{
	if (pDiffHull)
	{
		return pDiffHull[i];
	}
	else
	{
		return NULL;
	}
}

//////////////////////////////////////////////////////////////////////////////
// CWorkspaceCrosssection
//////////////////////////////////////////////////////////////////////////////

CWorkspaceCrosssection::CWorkspaceCrosssection(CRobotData& robot) 
: CWorkspaceAlgorithm(robot)
{
	axis=4;		// by default we use the z-axis (third bit) for the cross section grid direction
}

//! \todo currently, the line segments are model as triangles between the points (A,B,A)
//! Implement a more appropriate data model for lines       
void CWorkspaceCrosssection::makeCircleSegment(const Vector3d &v1, const Vector3d &v2, int depth)
{
    if (depth==0)
    {
		vertices.push_back(v1);
		vertices.back().normalize();
        vertices.push_back(v2);
		vertices.back().normalize();
		pair<int,int> p;
		p.first=vertices.size()-2;
		p.second=vertices.size()-1;
		Lines.push_back(p);
    }
    else
    {
        depth--;
        Vector3d v12=(v1+v2);v12.normalize();
        makeCircleSegment(v1,v12,depth);
        makeCircleSegment(v12,v2,depth);
    }
}

//! create the unit circle for any normal direction, where the z-axis of the 
//! orthogonal matrix normal_transformation is used as axis. any number of steps can be used.
//! return true, if the normal direction were successfully created. otherwise false
bool CWorkspaceCrosssection::makeCircle(const Matrix3d& normal_transformation, const int& steps) 
{
	if (steps<0)
		return false;
	
	for (int i=0; i<steps; i++)
	{
		// scale the number of steps to an angle in [0, 360°]
		double phi = 2.0 * MO_PI * (double)i / (double)steps ;
		Vector3d u(cos(phi),sin(phi),0);
		vertices.push_back(normal_transformation*u);
	}
	return true;
}

//! create an unit cirle around the origin perpendicular to the axis normal
void CWorkspaceCrosssection::makeCircle(const char& normal)
{
	int start_idx=vertices.size();
	// clean up memory
	switch (normal)
	{
	case 'x':
		makeCircleSegment(Vector3d(0, 1, 0),Vector3d(0, 0, 1),IterationDepth);
		makeCircleSegment(Vector3d(0, 0, 1),Vector3d(0,-1, 0),IterationDepth);
		makeCircleSegment(Vector3d(0,-1, 0),Vector3d(0, 0,-1),IterationDepth);
		makeCircleSegment(Vector3d(0, 0,-1),Vector3d(0, 1, 0),IterationDepth);
		break;
	case 'y':
		makeCircleSegment(Vector3d( 1, 0, 0),Vector3d( 0, 0, 1),IterationDepth);
		makeCircleSegment(Vector3d( 0, 0, 1),Vector3d(-1, 0, 0),IterationDepth);
		makeCircleSegment(Vector3d(-1, 0, 0),Vector3d( 0, 0,-1),IterationDepth);
		makeCircleSegment(Vector3d( 0, 0,-1),Vector3d( 1, 0, 0),IterationDepth);
		break;
	case 'z':
		makeCircleSegment(Vector3d(1,0,0),Vector3d(0,1,0),IterationDepth);
		makeCircleSegment(Vector3d(0,1,0),Vector3d(-1,0,0),IterationDepth);
		makeCircleSegment(Vector3d(-1,0,0),Vector3d(0,-1,0),IterationDepth);
		makeCircleSegment(Vector3d(0,-1,0),Vector3d(1,0,0),IterationDepth);
//		makeCircle(Matrix3d::Identity());
		break;
	default:
		return;
	}
	// move the circle to center
	for (unsigned int i=start_idx; i<vertices.size(); i++)
		vertices[i]+=center;
}

/*! calculate a cross-section of the workspace around center
 *  with in the plane given through the normal
 */
void CWorkspaceCrosssection::calculateWorkspaceCrosssection(const char& axis)
{
	time_t start=clock();
	Lines.clear();
	vertices.clear();
	makeCircle(axis);

	unsigned int i;
	for (i=0; i<vertices.size(); i++)
		lineSearch(vertices[i],0,searchMax);

	calculationTime = (int)(clock()-start);
}

/*! calculates a grid of cross section in each of the three coordinate planes
 *  the lower limit for steps is 3
 */ 
void CWorkspaceCrosssection::calculateWorkspaceCrosssectionGrid(const Vector3d& minBB, const Vector3d& maxBB, const unsigned int steps)
{
	// check input parameter
	// \todo

	// push the variable center
	Vector3d old_center=center;

	time_t start=clock();
	Lines.clear();
	vertices.clear();

	unsigned int i=0;	// we use only one index counting for all loops because otherwise we will overwrite the results from the first calcuations
	// sucsessive calls to makeCircle will continuously extend the size of the vertices data structure
	// create the x-normal cross sections
	if (axis&1)
	{
		center=0.5*(minBB+maxBB);
		for (center.x()=minBB.x(); center.x()<=maxBB.x(); center.x()+=(maxBB.x()-minBB.x())/steps)
		{
			makeCircle('x');
			for (; i<vertices.size(); i++)
				lineSearch(vertices[i],0,searchMax);
		}
	}

	// create the y-normal cross sections
	if (axis&2)
	{
		center=0.5*(minBB+maxBB);
		for (center.y()=minBB.y(); center.y()<=maxBB.y(); center.y()+=(maxBB.y()-minBB.y())/steps)
		{
			makeCircle('y');
			for (; i<vertices.size(); i++)
				lineSearch(vertices[i],0,searchMax);
		}
	}

	// create the z-normal cross section
	if (axis&4)
	{
		// create the z-normal cross sections
		center=0.5*(minBB+maxBB);
		for (center.z()=minBB.z(); center.z()<=maxBB.z(); center.z()+=(maxBB.z()-minBB.z())/steps)
		{
			makeCircle('z');
			for (; i<vertices.size(); i++)
				lineSearch(vertices[i],0,searchMax);
		}
	}

	// pop the value of center
	center=old_center;
	// determine calculation time
	calculationTime = (int)(clock()-start);
}

/*! experimental exporting of contours to SVG file format
 *  \param [in] filename
 *  \param [in] normal must be 'x', 'y' or 'z'. selection the direction of projection
 *         for the 3D to 2D mapping needed to make a 2D svg plot
 *  \return true, if successful, otherweise false
 */
bool CWorkspaceCrosssection::saveWorkspaceCrosssection(const string& filename, const char normal)
{
	ofstream file(filename.c_str());
	// write the svg header
	file << 
		"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
		"<svg xmlns=\"http://www.w3.org/2000/svg\"\n"
		"    xmlns:xlink=\"http://www.w3.org/1999/xlink\"\n"
		"    xmlns:ev=\"http://www.w3.org/2001/xml-events\"\n"
		"    version=\"1.1\" baseProfile=\"full\"\n"
		"    width=\"400\" height=\"300\" viewBox=\"0 0 400 300\">\n";

	file << "<rect x=\"-200\" y=\"-50\" width=\"400\" height=\"250\" fill=\"white\" stroke=\"black\" stroke-width=\"3px\" />\n";

	const double scale = 100;
	for (unsigned int i=0; i<Lines.size(); i++)
	{
		switch (normal)
		{
			case 'x':
				file << "<line x1=\"" << vertices[Lines[i].first].y()*scale << "\" y1=\"" << vertices[Lines[i].first].z() *scale 
					<< "\" x2=\"" << vertices[Lines[i].second].y() *scale << "\" y2=\"" << vertices[Lines[i].second].z() *scale
					<< "\" fill=\"none\" stroke=\"black\" stroke-width=\"3px\"/>\n";
				break;
			case 'y':
				file << "<line x1=\"" << vertices[Lines[i].first].x()*scale << "\" y1=\"" << vertices[Lines[i].first].z() *scale 
					<< "\" x2=\"" << vertices[Lines[i].second].x() *scale << "\" y2=\"" << vertices[Lines[i].second].z() *scale
					<< "\" fill=\"none\" stroke=\"black\" stroke-width=\"3px\"/>\n";
				break;
			case 'z':
				file << "<line x1=\"" << vertices[Lines[i].first].x()*scale << "\" y1=\"" << vertices[Lines[i].first].y() *scale 
					<< "\" x2=\"" << vertices[Lines[i].second].x() *scale << "\" y2=\"" << vertices[Lines[i].second].y() *scale
					<< "\" fill=\"none\" stroke=\"black\" stroke-width=\"3px\"/>\n";
				break;
		}
	}

	file <<	"</svg>\n";
	return true;
}

//! this is an experimental raw output of the internal data model to be used
//! by 3rd party software as input to create diagrams. The maple output
//! works but is slow and maple does not allow to export the diagram as
//! vector graphics. Anyway, it works as a quick but dirty solution.
bool CWorkspaceCrosssection::saveWorkspaceCrosssectionMatlab(const string& filename)
{
	// legacy maple writer; this should be encapsulatd into another function
	// maple mode
/*	ofstream file(filename.c_str());
	file << "with(plots); with(plottools):\n"; 
	file << "RAW:=[ ";
	for (unsigned int i=0; i<Lines.size(); i++)
	{
		file << "line(["   << vertices[Lines[i].first].x() 
			 << ","   << vertices[Lines[i].first].y() 
			 << ","   << vertices[Lines[i].first].z() 
			 << "],[" << vertices[Lines[i].second].x() 
			 << ","   << vertices[Lines[i].second].y() 
			 << ","   << vertices[Lines[i].second].z()
			 << "]),\n";
	}
	file << "]:\n";
	file << "display(RAW,axes=boxed,labels=[x,y,z]);\n";*/

	// save in matlab mode
	ofstream mfile((filename+".m").c_str());
	// write all vertex-Data for start-end points of the lines
	mfile << "DAT=[ ";
	for (unsigned int i=0; i<Lines.size(); i++)
		mfile << vertices[Lines[i].first].x() << " "
			 << vertices[Lines[i].second].x() << " "
			 << vertices[Lines[i].first].y() << " "
			 << vertices[Lines[i].second].y() << " "
			 << vertices[Lines[i].first].z() << " "
			 << vertices[Lines[i].second].z() << ";\n";
	mfile << "]';\n";

	// write the plotting command and configure the plot
	mfile << "figure(1)\n"
		<< "axes('FontSize',16)\n"
		<< "patch(DAT(1:2,:),DAT(3:4,:),DAT(5:6,:),1,'LineWidth',2)\n"
		<< "axis ([-2 2 -1.5 1.5 0 2])\n" // limit the axis to the defined values
		<< "grid on\n"
		<< "box on\n"
		<< "xlabel('x')\n"
		<< "ylabel('y')\n"
		<< "zlabel('z')\n"
		<< "view([35 30])\n"		// set the rotation angle for the 3d plot
		<< "print -depsc " << filename << "\n"	// auto export to eps
		<< "%close(1)\n";			// add a command to close the figure 
	return true;
}

//! get the compute cross section as a matrix with with a dimension of 6 by n 
//! if the function returns true, cs containts the start and end points of the
//! a polygon as columns of the matrix cs
bool CWorkspaceCrosssection::getWorkspaceCrosssectionMatrix(MatrixXd& cs)
{
	if (Lines.size()==0)
		return false;

	cs.resize(6,Lines.size());
	for (unsigned int i=0; i<Lines.size(); i++)
	{
		cs.block(0,i,3,1) = vertices[Lines[i].first];
		cs.block(3,i,3,1) = vertices[Lines[i].second];
	}
//	cout << cs;
	return true;
}


} // end of namespace PCRL