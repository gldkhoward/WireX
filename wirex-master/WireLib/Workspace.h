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

/*! \file Workspace.h
 *
 *	\author   Andreas Pott
 *
 *  \dependency
 *		Eigen3		for IR³ algebra
 *
 *  \todo Draft - Interface and inner structure (draft for a major update of the class)
 *  The following three aspects in worksapce calculation should be fully decoupled:

 *  1) The following three main driver functions
 *  - calculate: initial determination of the workspace
 *  - intersect: calculate intersection of a criterion with the already determined workspace
 *  - union: extend the already determined workspace with a criterion
 *  Basically this lists seems to be complete. A natural extension allows to reduce the calcuation to subset of vertices 
 *  (for partical analysis or for parallel processing)
 *
 *  2) Implement different calculation schemes based on the kind of workspace one is interested in:
 *  - constant orientiation workspace or translational workspace (use one given orientation R)
 *  - maximum workspace or reachable workspace: true if at least one R_i in SO_3 is possible
 *  - inclusion orientation workspace: use a given set of orientations R_i; true if at least one R_i is possible 
 *  - total orientation workspace: use a given set of orientations R_i; true if all R_i are possible 
 *  - dextrous workspace: same as total orientation workspace where the given set is the whole group SO_3
 *  These types of workspace are taken from JP Merles book "Parallel Robots" 
 *  and the list is expected to be complete.
 * 
 *  3) The criteria applied to given pose for evaluation
 *  - Existing Force distribtion (wrench-feasible/controllable workspace); different methods may be possible to evalutate
 *  - available wrench set
 *  - length of the cables feasible
 *  - collision issues 
 *  -- cable-cable
 *  -- cable-frame 
 *  -- cable-platform
 *  -- cable-obstacle
 *  - kinematic issues of the winch or platform (limitations e.g. in the movability of the pulley)
 *  
 *  Therefore, the refacturing shall decouple the three layers. Where the 
 *  evaluation layer should be extendable to integrate future workspace criteria 
 *  (such as singularies)
 */

#pragma once

#include <string>
#include <vector>
#include "StructureMatrix.h"
#include <motionPlanning/Utilities.h>

namespace PCRL { // namespace Parallel Cable Robot Library

/*! \class CWorkspaceAlgorithm
 *  \brief Base class for workspace calculation algorithms.
 *  Cable robot workspace analysis is based on different criteria that can
 *  be plugin to the class be a number of slots which are of type
 *  CPoseProperty.
 */
class CWorkspaceAlgorithm : public CAlgorithm
{
protected:
	void bind();
	//! algorithm parameter
	int IterationDepth;		//!< number of recursive subdivison for hull generation
	double eps;				//!< accuracy for line search
	double searchMax;		//!< maximum distance for the line search
	bool allOrientations;   //!< defines if a position is rated feasible if one or all orientations in the orientation set are feasibile
	Vector3d minBB,maxBB;	//!< clipping box for workspace determination
	Vector3d center;		//!< estimation of the center of the workspace

	int calculationTime;	//!< calculation time for workspace determination in ms

	//! plug-in interface for implementations checking workspace criteria
	CPoseProperty* pForceDistribution;	//!< the plug-in slot for force computation algorithms
	CPoseProperty* pKinematics;			//!< the plug-in slot for kinematics testing
	CPoseProperty* pWrenchSet;			//!< plut-in slot for the wrench capability test
	CPoseProperty* pVelocitySet;		//!< plug-in slot for the velocity (twist) capability test
	CPoseProperty* pStiffness;          //!< plug-in slot for stiffness evaluation
public:
	typedef enum {forceFeasible=0, wireLength=1, boundingBox=2, wrenchSet=3, velocitySet=4, stiffness=5, forceClosure=6} eWorkspaceCriterion;
protected:
	eWorkspaceCriterion method; 

	//! new implementation for the line search approach
	bool testPose(const Vector3d& p0, const Matrix3d& Orientation);
	
	//! perform a line search beginning at center towards v
	void lineSearch(Vector3d& v, double lambda_min, double lambda_max);

	//! an internal set of orientation to be checked at a single position
	//! the orientation for evaluation of the constant orientation workspace (if a single orientation has to be checked)
	vector<Matrix3d> OrientationSet;

public:
	explicit CWorkspaceAlgorithm(CRobotData& robot);

	//! test a position to belong to the workspace with the current setting for the orientation 
	bool workspaceEvaluator(const Vector3d& p0);

	//! perform a line search in the orientaton direction 
	double lineSearchAngular(const Vector3d& r, const Vector3d& u, double lambda_min, double lambda_max);

	//! connect the plug-ins for the evaulator objects
	void attachEvaluator(CPoseProperty& ForceDistribution, CPoseProperty& Kinematics, CPoseProperty& WrenchSet, CPoseProperty& VelocitySet, CPoseProperty& Stiffness)
		{ pForceDistribution=&ForceDistribution; pKinematics=&Kinematics; pWrenchSet=&WrenchSet; pVelocitySet=&VelocitySet; pStiffness=&Stiffness;}

	//! calculate the aperture of the cone around axis u and apex at a_i that fully covers a workspace box given through (min,max)
	double getWinchAperture(const int& id, const Vector3d& min, const Vector3d& max, const Vector3d& u);

	//! set center for projects in workspace calculation
	void setProjectionCenter(const Vector3d& Center) { center=Center; }
	//! get the center for projects in workspace calculation component wise
	void getProjectionCenter(double &x, double &y, double &z) const { x=center.x(); y=center.y(); z=center.z(); }
	//! get the center for projects in workspace calculation as vector
	void getProjectionCenter(Vector3d& Center) const { Center=center; }
	//! set the bounding box for the clipping test
	void setBoundingBox(const Vector3d& MinBB, const Vector3d& MaxBB);
	//! set the depth for recursive subdevision in workspace computation
	void setIterations(const int Iterations) { IterationDepth=Iterations;}
	//! return the current iteration depth
	int getIterations() const { return IterationDepth; }
	//! set the accuracy for recursive line search algorithms
	void setEps(const double& Eps) { eps=Eps; }
	//! return the current value for the accuracy of the line search algorithm
	double getEps() const { return eps; }
	//! set workspace criterion
	void setWorkspaceCriterion(eWorkspaceCriterion Method) { method=Method; }
	//! get workspace criterion
	int getWorkspaceCriterion() const { return method; }
	//! set the maximum range of the line search algorithm
	void setSearchRange(const double& Range) { searchMax=Range; }
	//! select the requirement for a pose to belongs to the workspace; true means all orientations in the set needs to be satisfied; false means at least one orientation in the set is sufficient
	void setOrientationRequirement(bool b) { allOrientations=b; }
	//! get the value of the orientation requirement 
	bool getOrientationRequirement() { return allOrientations; }
	//! get the computation time in ms (may depend on computer platform) of the last workspace computation
	int getCalculationTime() const { return calculationTime; } 
	//! set the value of calcuationTime. 
	void setCalculationTime(const int time) { calculationTime=time; } 
	//! return the current search range for the line search
	double getSearchRange() const { return searchMax; }
	//! copy algorithm parameters from another object
	void getAlgorithmParameter(const CWorkspaceAlgorithm& src);
	//! define the orientation of the mobile platform for workspace calculation
	void setOrientation(const Matrix3d& R) { OrientationSet.clear(); OrientationSet.push_back(R); }
	//! return the current list of orientations 
	vector<Matrix3d> getAllOrientations() { return OrientationSet; }
	//! return the orientation matrix from a special position
	Matrix3d getOrientation(int index) { return OrientationSet.at(index); }
	//! add one orientation to the list 
	void addOrientation(const Matrix3d& R) { OrientationSet.push_back(R); }
	//! create orientation set
	bool createOrientationSet(double delta_a=0, double delta_b=0, double delta_c=0, const unsigned int& steps=10, const Matrix3d& R0=Matrix3d::ZRotationMatrix3d(0)); 
	//! create orientation set with common angle aperture for all the axes
	bool createSymmetricOrientationSet(double delta_x=0, const unsigned int& steps=10, const Matrix3d& R0=Matrix3d::ZRotationMatrix3d(0)); 
	//! add random orientation to the orientation set
	bool createRandomOrientationSet(const int count, Matrix3d& R0, const double& max_angle=2*MO_PI);
	//! get the number of elements in the orientation set
	int getOrientationCount() const { return OrientationSet.size(); }
};

/*! \class CWorkspaceHull
 *  Calculate the cartesian hull of the constant orientation or total 
 *  orientation workspace workspace by expanding a unit sphere until 
 *  it reaches the boundary of the workspace. Obviously, this approach 
 *  is based on assumptions on the geometry of the workspace that are not 
 *  fulfilled in general. However, this simplification works well for robots 
 *  with reasonable design parameters.
 */
class CWorkspaceHull : public CWorkspaceAlgorithm 
{
public:
	//! workspace hull data model
	vector<Vector3d> vertices;				//!< indexed array of all mesh points (vertices) of the workspace hull
	vector<CTriangleIndices> Triangles;		//!< array of tripplets with the ids of the vertices to be connected to a triangle
	vector<CEdge> edges;					//!< array of pairs of vertices being the edges of the grid
	unsigned int roiStart,roiEnd;	//!< allows to restrict the workspace computation to certain region
	int FinishedTriangles;			//!< index of last vertex which data have been computer
	int currentVertex;
	int currentTriangle;
	MatrixXd CableSpanMatrix;			//!< this matrix contains the cable span for all cables in polar coordinates
	MatrixXd CableSpanTriangulation;	//!< this matrix contains the cable span in Cartesian coordinates 

public:
	//! statistic information about the workspace hull
	double Volume;		//!< volume of the workspace m^3
	double Surface;		//!< surface area of the workspace m^2
	Vector3d CoI;		//!< vector to the center of inertia
	Vector3d bbMin;		//!< minimum vector of the axis-aligned bounding box
	Vector3d bbMax;		//!< maximum vector of the axis-aligned bounding box
	bool bCenterValid;		//!< a boolean flag indicating if the pose center itself is valid or not

	//! delete the workspace data and replace it with a triangulized unit sphere
	void makeSphere();

protected:
	// workspace algorithms
	void makeSpherePatch(unsigned int edgeID0, unsigned int edgeID1, unsigned int edgeID2, int depth);
	void divideVertices();

public:
	explicit CWorkspaceHull(CRobotData& robot);
	virtual ~CWorkspaceHull();

	//! calculate the optimal cone for a given apex so that the cone contains the whole robot workspace
	//! public function
	void calculateOptimalCone(const Vector3d& apex, Vector3d& axis_out , double& aperture_out);
	//! calculate the optimal cone for a specific pivot
	void calculateOptimalCone(const unsigned int id, Vector3d& axis_out , double& aperture_out);

	//!  calculate disabled workspace, i.e. the workspace not reachable in the presents of a box given by vBox_min -- vBox_max
	void calcDisabledWspc(Vector3d& vBox_min, Vector3d& vBox_max);
	
	//!  test if vertex lies in the shadow of a box
	bool inShadowArea(Vector3d& vBox_min, Vector3d& vBox_max, Vector3d& vertex);
	
	//! determine volume and surface of workspace
	bool calculateWorkspaceProperties();
	bool calculateWorkspaceProperties(double &Volume, double &Surface, Vector3d &CoI);

	//! print some statistics on the screen
	void printProperties() const;

	//! save the workspace in an STL file
	bool saveWorkspace(const string& filename);
	//! save the workspace hull as Matlab script to generate a diagram
	bool saveWorkspaceMatlab(const string& filename);
	//! save the workspace vertices to a csv file
	bool saveVerticesCSV(const string& filename);
	//! load the workspace vertices from a csv file
	bool loadWorkspaceCSV(const string& filename, int iIterationDepth);

	//! Return the data of the workspace hull
	bool getWorkspaceGeometry(vector<Vector3d> &vertices_out, vector<CTriangleIndices> &triangles_out, int &finishedTriangles_out);
	//! save the space occupied by the cables to an STL file
	bool saveCableSpaceEnvelope(const string& filename, int cables=-1);
	//! get the vector from center to the hull surface
	Vector3d getVertexRay(const unsigned int& id);
	//! get the length of the line from center to the workspace hull
	double getVertexRayLength(const unsigned int& id);
	//! driver functions 

	//! compute the workspace 
	bool calculateWorkspace(bool initializedByLast = false);
	//! intersection (logical AND) the result of a workspace computation with the workspace currently stored in the internal buffers
	bool intersectWorkspace();
	//! expend (logical OR) the result of a workspace computation with the workspace currently stored in the internal buffer
	bool uniteWorkspace();
	bool calculateWorkspaceCore(bool initializedByLast = false);

	//! reduce the workspace to a box if the vertices of the workspace exceed the box
	bool clipByBoundingBox(const Vector3d& bbmin, const Vector3d& bbmax);
	
	//! Bloat the hull by length of deltaLength undirectional to the surface normales
	bool bloatHullBy(double deltaLength);

	//! helper functions and objects to enable parallel prozessing of workspace
public:
	//! copy the hull vertices beginning from start to end from the source object
	bool copyVertexData(const CWorkspaceHull& src, const unsigned int& start, const unsigned int& end);
	//! set the range of vertices to be computed by the algorithm
	bool setVertexRoi(const unsigned int& start, const unsigned int& end);

	//! compute an approximation of the workspace span from the current workspace
	bool calculateCableSpan();
	// the following function block is a draft interface for parallel computation
	// til now, no implementation exists
//private:
//	unsigned int noc;		//!< number of parallel workspace objects
//	CWorkspaceHull** pNodes;//!< pointer to the nodes
//public:
	//! create a couple of clone objects to parallel processing of the vertices
//	bool createTask(const unsigned int noc);
	//! distribute the workload through the objects
//	bool spreadTask(const unsigned int noc, CWorkspaceHull* parallizedObjects);
	//! start the i-th job
//	bool startTask(const unsigned int id);
	//! collect the data from a set of parallized workspace objects (that were generated through spreadTasks)
//	bool collectTaskResults(CWorkspaceHull* parallizedObjects);
	//! dispose the set of objects generated by createTask
//	bool disposeTask();

private:
	//! computes the optimal cone for a specific pivot so that the cone contains the whole robot workspace
	//! private helper functions
	void calculateOptimalCone_(vector<unsigned int> vID, vector<unsigned int> bID, const Vector3d& apex, Vector3d& axis_out , double& aperture_out);
	bool isInsideCone(const unsigned int& p_i, const Vector3d& apex, const Vector3d& axis , const double& aperture);
};


//! combination of a point and a bool variable signaling if the point belongs to the workspace
class CPointFeasible 
{
public:
	Vector3d point;	//!< position
	bool bFeasible;	//!< belongs to the workspace

};

/*!	\class CWorkspaceGrid
 *  This class checks the workspace based on a discreate, predefined distribution 
 *  of poses.
 *  Workspace calculation is done in a two stage process: In the first stage
 *  the grid is generated and in the second step each pose within the grid is
 *  evaluated. The typical procedure for generating the grid is to evenly distribute
 *  the poses in regular chess board like grid. Additionally, methods can be
 *  used to create other distributions.
 *  The data model for the grid is a vector of poses called "vertices". During 
 *  workspace computations the positions in the vector vertices are distributed 
 *  amongst the vectors workspace and workspace_out.
 *
 *  \todo Add other forms of grid, e.g. spherical coordinates / (concentric) spheres, 
 *        (concentric) cyclinders, hexagonal grids, 
 */
class CWorkspaceGrid : public CWorkspaceAlgorithm
{
	//! helper function to generate a random number between lower and upper
	double randomNumber(const double& lower, const double& upper);

public:
	//! grid data model
	vector<Vector3d> vertices;
	//! position that actually belong to the workspace
	vector<Vector3d> workspace;
	//! position that actually not belong to the workspace
	vector<Vector3d> workspace_out;
	//! grid data model with the information if the point belongs to the workspace or not
	vector<CPointFeasible> vertices_expanded;
	//! number of discrete elements in x, y and z direction
	int i_max, j_max, k_max;
	//! check the internal points of the inner box (not only the edges)
	bool bCheckInternalPoints; 
	//! edges of the inner box
	Vector3d bbmin, bbmax;
public:
	explicit CWorkspaceGrid(CRobotData& robot);
	virtual ~CWorkspaceGrid();

	//! create a regular grid of poses to be checked
	bool makeGrid(const Vector3d& min, const Vector3d& max, const Vector3d& eps=Vector3d(0,0,0));
	//! create a regular grid of poses to be checked
	bool makeRandomGrid(const Vector3d& min, const Vector3d& max, const unsigned int size);
	//! create a positions within the box min/max with Gaussian normal distributions
	bool makeGaussianGrid(const Vector3d& min, const Vector3d& max, const Vector3d& sigma, const unsigned int size);
	//! create a grid with concentric cyclinders 
	bool makeCylinderGrid(const Vector3d& min, const Vector3d& max, char normal='z', int layerSteps=10, int radialSteps=10, int AngularSteps=36);
	//! filter the positions that belong to the workspace
	bool calculateWorkspace();
	//! calculate the orientation workspace by sampling through the orientations in orientationSet
	bool calculateOrientationWorkspace(const Vector3d& r, double& coverage);
	//! get the coverage between the verified points of the workspace and the size of the grid
	double getCoverage() const { return vertices.size()!=0?( (double)workspace.size() / (double)vertices.size() ) : 0.0; }
	//!< calcualte and return the orientation workspace coverage
	double getOrienationWorkspaceCoverage(const Vector3d& r);
	//! export the internal grid as pose list
	// bool getPoseList(CPoseListStatic& poseList);
	//!\ todo Refactor the following function to use the grid generation provided by the class (makeGrid)
	//! calculate the maximum and minimum wire forces for a given box
	bool wireForcesWorkspaceBox(MatrixXd& Fmin, MatrixXd& Fmax, const Vector3d& Min, const Vector3d& Max, 
		const Matrix3d& R, const Vector3d& eps=Vector3d(0,0,0));
	//! driver function for calculating the maximum and minimum wire forces for a given box
	bool wireForcesWorkspaceBoxDriver(const Vector3d& min, const Vector3d& max, 
		const Vector3d& eps=Vector3d(0,0,0));
	//! verify if a given box belongs to the workspace
	bool verifyWorkspaceBox(const Vector3d& min, const Vector3d& max, const Vector3d& eps=Vector3d(0,0,0));
	//!determine the box with the biggest volume which lies inside the workspace
	bool calculateInnerBox();
	//! check the internal points of the inner box (not only the edges)
	void setCheckInternalPoints(bool bCheckInternalPoints_) { bCheckInternalPoints=bCheckInternalPoints_; return; }
	//! verify if a given box belongs to the workspace
	bool verifyWorkspaceBox(const double& minx, const double& miny, const double& minz, 
		const double& maxx, const double& maxy, const double& maxz, 
		const double& epsx=0, const double& epsy=0, const double& epsz=0)
		{ return verifyWorkspaceBox(Vector3d(minx,miny,minz),Vector3d(maxx,maxy,maxz),Vector3d(epsx,epsy,epsz)); }
	//! save WSGrid to ascii text file
	bool saveWSGrid(const string& filename);
};

/*! \class CWorkspaceDifferentialHull
 *  Calculate the differential influence of changes in the geometry of the robot
 *  on the size and shape of the workspace. Therefore, a finite differences
 *  method is applied to approximate the change in length of the surface of the
 *  robot as well as to determine the differential change in the derived properties
 *  such as volume, surface, and bounding-box. 
 *  The nominal workspace is stored in the data strcutre inherited from CWorkspaceHull.
 */
class CWorkspaceDifferentialHull : public CWorkspaceHull
{
	friend class CShapeWorkspaceDiffHull;
protected:
	double epsGeometry;				//!< stepsize for change in the geometry for finite differences
	int nHulls;						//!< the number of hull objects reserved
	CWorkspaceHull** pDiffHull;		//!< the differences calculate for the variated parameter sets
	MatrixXd parameterDifferentials;//!< a matrix with all the differentials. rows relate to parameters varied, cols relate to differentials of properties
public:
	explicit CWorkspaceDifferentialHull(CRobotData& robot);
	~CWorkspaceDifferentialHull();
	
	//! calculate the differential workspace
	bool calculateWorkspaceDiff();
	//! collect the results in a matrix
	bool calculateParameterDifferentials();
	//! return the result matrix
	MatrixXd getParameterDifferentials() const;
	//! print some of the calculated data to the screen
	void printDifferential();
	//! set the eps for finite differences
	void setEpsGeometry(const double& eps) { epsGeometry = eps; }
	//! get the eps for finite differences
	double getEpsGeometry() const { return epsGeometry; }
	//! returns the selected differential hull
	CWorkspaceHull* getDifferntialHull(int i);
};

/*! \class CWorkspaceCrosssection
 *  Calculate the one-dimensional boarder of a cross secton of the workspace 
 *  by expanding a unit circle in one of the coordinate frame's planes until 
 *  it reaches the boundary of the workspace.
 */
class CWorkspaceCrosssection : public CWorkspaceAlgorithm 
{
public:
	// workspace hull data model
	//! the vertices of the cross section	
	vector<Vector3d> vertices;
	//! indices for pairs of vertices to be connected by a line
	vector<pair<int,int> > Lines;
	//!< bitfield indicating which normal axis are used for the generation crosssections (x=1, y=2, z=4; all axis->7)
	int	axis;		
protected:
	//! recursive function to refine the vertex data of the cross section
	void makeCircleSegment(const Vector3d &v1, const Vector3d &v2, int depth);
	//! driver function that generates a unit circle with normal around the current center
	void makeCircle(const char& normal='z');
	//! generate the circle for a normal direction and preorientation specified by a matrix
	bool makeCircle(const Matrix3d& normal_transformation, const int& steps=36);
public:
	explicit CWorkspaceCrosssection(CRobotData& robot);
	//! calculates a cross section of the workspace
	void calculateWorkspaceCrosssection(const char& normal='z');
	//! calculates a grid of cross section in the coordinate planes defined in the bitfield 'axis'
	void calculateWorkspaceCrosssectionGrid(const Vector3d& min, const Vector3d& max, const unsigned int steps=10);
	//! save a cross-section as SVG file
	bool saveWorkspaceCrosssection(const string& filename, const char normal='z');
	//! save the 3d-cross section data to a file, e.g. to be used by other programs for diagrams
	bool saveWorkspaceCrosssectionMatlab(const string& filename);
	//! get the compute cross section as a matrix with with a dimension of 6 by n 
	bool getWorkspaceCrosssectionMatrix(MatrixXd& cs);
};

} // end namespace PCRL
