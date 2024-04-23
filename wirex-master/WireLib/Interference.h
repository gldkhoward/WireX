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

/*! \file Interference.h
 *
 *	\author   Andreas Pott
 *
 *  \dependency
 *		Eigen3		for IR³ algebra
 *
 *  \brief 
 *  This class CInterference implements algorithms both for local and global
 *  collision tests. The main algorithm tests cable-cable interference computing
 *  all poses that are affected by collision amongst the cables. Using the 
 *  configuration parameters, one can extend the collision test from the constant
 *  orientation workspace to the orientation workspace. 
 *  Furthermore, some pose dependent test can be executed to explicitly check a 
 *  given configuration for interference.
 */

#pragma once

#include <vector>
#include <string>
#include "Algorithm.h"
#include "EigenLib.h"

namespace PCRL {

/*! \class CInterference 
 *  Calculate the regions of cable-cable interference
 */
class CInterference : public CAlgorithm 
{
	friend class CShapeInterference;
	friend struct sortstruct;			//!< hotfix but better than public members

protected:
	//! robot model (geometry, technological parameters)
	vector<Vector3d> triangles;
	vector<Vector3d> lines;

	double xmin, xmax, ymin, ymax, zmin, zmax; //!< cliping range for cable intersection test
	
	//! interval and step width for discretisation of orientation workspace in cable intersection test
	double alphaMin,alphaMax,deltaAlpha;
	double betaMin,betaMax,deltaBeta;
	double gammaMin,gammaMax,deltaGamma;

	double TriangleLength;	//!< indicate the size of the triangle region to be considered (theoretically infinity)
	double eps;				//!< threshold for angle between two lines to be considered as parallel
	Matrix3d R0;			//!< the reference orientation. the variations with alpha/beta are choosen around this orientation; R0=I by default
	void bind();			//!< implement the parameter binding 

	int firstCable, lastCable;	//!< the first and last cable to be included in the collision check; use these parameters to restrict the test to a subset of cables

	MatrixXd cableCableCollisions;			//!< 0, if cable-cable collisions		/ distance or Eigen::Infinity, otherwise
	MatrixXd cableCableBoxCollisions;		//!< 1, if cable-cable collisions		/ 0, otherwise
	MatrixXd cablePlatformCollisions;		//!< 1, if cable-platform collisions	/ 0, otherwise
	MatrixXd cablePlatformBoxCollisions;	//!< 1, if cable-cable collisions		/ 0, otherwise

	// New internal variables
	double etaMax;		//!< maximal angle of rotation matrices in the orientation set for the platform
	
	double deltaalpha; //!< parameter controling the discretization of the cable-cable interference sets
	double deltaeta;   //!< parameter controling the discretization of the cable-cable interference disc and sphere cap subsets
	int stepscircle;   //!< parameter controling the discretization of the cable-cable interference disc and sphere cap subsets
	bool displayAllCableCableInterferenceSets;  //!< display option for cable-cable collision

	int printParaVisNumber;     //!< parameter to select printable parallelogram to be visualized
	Vector3d pn;		        //!< vector from the center of the platform to the head of the printing nozzle
	Vector3d printcenter;       //!< center of printed shapes
	Vector3d d1,d2;             //!< direction vectors for printed superellipses and parallelograms
	double pEllipse;            //!< exponent parameter of superellipse
	bool displayshadows;        //!< option to display shadows
	double printheight;         //!< determines hight of printed 3d object
	int numberofprintlayers;    //!< number of printing layers in height interval
	vector<int> notexpandable;  //!< index list for calculating multiple parallelogram layers

	Vector3d platformcenterposition; //!< center of cable-platform collision free workspace
	int refinement,searchsteps;      //!< parameters for cable-platform collision calculations
	
	Vector3d printcolor;					 //!< color of printable shapes
	Vector3d shadowcolor;					 //!< color of shadow shapes

	vector<Vector3d> shadowcenters;			 //!< list of shadow centers cast by printcenter
	vector<double> shadowscalings;			 //!< list of shadow scaling factors
	double printellipsesize;				 //!< size of largest printable superellipse around printcenter
	vector<Vector3d> spokelist;				 //!< list of spokes of the printable star-shaped polygon around printcenter
	
	vector< vector<int> >printparasizes;     //!< list of printable parallelogram sizes around printcenter
	vector< vector<double> >multparasizes;   //!< list of parallelogram sizes around printcenter in multiple layers
	bool multlayermode;						 //!< parameter indicating that multiple parallelogramm layers will be calculated

	vector< vector<Vector3d> > cablecablepartitionedpointlist;         //!<    point list for convex hulls of cable-cable collision sets
	vector< vector< vector<int> > > cablecablepartininedtrianglelist;  //!< triangle list for convex hulls of cable-cable collision sets

	vector<Vector3d> redtriangles;			         //!< list of red triangle points to be visualized by Shapes.h    
	vector< vector<Vector3d> > printshapes;          //!< list of triangle fans to be visualized by Shapes.h
	vector< vector<Vector3d> > looplines;	         //!< list of black looping lines to be visualized by Shapes.h
	vector< vector<Vector3d> > layerconnections;	 //!< list of triangle strips to be visualized by Shapes.h

	vector<Vector3d> platformstldata;                  //!< list of triangle points froming the platform geomety
	vector<  vector< vector<Vector3d> >  > cablecones; //!< list of cable-platform collision cones
	vector<Vector3d> cableplatformraylist;             //!< list of cable-platform collision rays spanning the collision free position set
	vector< vector<int> > cableplatformtrianglelist;   //!< list of triangles connecting the cable-platform collision rays

	vector< vector<double> >charparasizes;   //!< list of characteristic parallelogram sizes around printcenter (needs to be public for sortstruct to work)
public:
	// --------------------------- New methods ---------------------------  

	//! compute rotation matrix around axis n with angle alpha
	Matrix3d NalphaRotation(const Vector3d& n, const double& alpha) const;

	//! update the clipping planes and the Trianglelength parameter according to the robot geometry
	void updateClippingPlanes();

	//! set maximal angle of rotation matrices in the orientation set for the platform
	void setEtaMax(double eta);

	//! write robot geometry into .csv file
	bool writeRobotGeometryToCSV(const string& filename);

	//! set direction vectors for printing parallelograms and superellipses
	void setPrintDir(Vector3d dir1,Vector3d dir2);
	
	//! set center location for printing parallelograms and superellipses
	void setPrintCenter(Vector3d center);
	
	//! set center location for cable-platform calculation
	void setPlatformCenter(Vector3d center);
	
	//! set exponent parameter for printing superellipses
	void setEllipseExp(double t);

	//! set printheight parameter for printing
	void setPrintHeight(double h);
	
	//! set refinement parameter for cable-platform ray data structure
	void setRefinement(int r);
	
	//! set searchsteps parameter for cable-platform calculation
	void setSearchSteps(int s);

	//! set printParaVisNumber parameter for selecting visualized printable parallelogram
	void setPrintParaVisNumber(int s);

	//! set numberofprintlayers parameter for selecting the number of layers in the printing height interval
	void setNumberOfPrintLayers(int s);

	//! set cable-cable display setting
	void setCableCableSetting(bool setting);

	//! set cable-print display setting
	void setPrintDisplayShadows(bool setting);

	//! set discretization parameter for cable-cable interference
	void setDeltaAlpha(double a);

	//! set discretization parameter for cable-cable interference
	void setDeltaEta(double h);

	//! set discretization parameter for cable-cable interference
	void setStepsCircle(int h);

	//! delete currently visualized shapes
	void clearShapes();


	// --------------------------- cable-cable --------------------------- 

	//! get minimal size of the orientation set for which cable-cable collisions can occur everywhere
	double calculateMinInterferenceAngle();
	
	//! compute points of cable-cable collision sets
	void setupCableCableInterferenceSets();
	
	//! calculate convex hull of points in in pointlist, starting with the first triangle in trianglelist
	void calculateConvexHull( vector<Vector3d>& pointlist, vector< vector<int> >& trianglelist);
	
	//! calculate cable-cable collision sets connecting two sphere cap sets
	void connectSphereSections(const Vector3d& bcur, const Vector3d& bijstep, const Vector3d& aj, const Vector3d& aij);
	
	//! calculate cable-cable collision sets connecting two discs 
	void connectSphereCircles(const Vector3d& bcur, const Vector3d& bijstep, const Vector3d& aj, const Vector3d& aij);
	
	//! calculate convex hulls for all cable-cable collision sets
	void calculateCableCableInterferenceSetsHulls();
	
	//! setup convex hulls to be rendered by Shapes.h
	void setupCableCableInterferenceHullsTriangles();
	
	//! write points of cable-cable collision sets into .csv file
	bool writeCableCableSetsToCSV(const string& filename);
	
	//! visualize cable-cable Interference sets
	void visualizeInterferenceCableCable();

	//! Check if a pose is valid with regards to cable-cable collisions
	bool checkPoseForCableCableCollision(const Vector3d& c, const Vector3d& r, const Matrix3d& R);

	//! visualizee calbe-platform interference
	void visualizeInterferenceCablePlatform(const string& filename);
	
	// --------------------------- cable-platform --------------------------- 
	
	//! load platform geometry in .stl format
	bool loadplatformstldata(const string& filename);
	
	//! visualize platform geometry
	void drawplatformstldata();
	
	//! compute collision cones at distal anchorpoints that encapsulate the platform
	void setupCablePlatformCollisionCones();
	
	//! check for cable-platform collisions at position x within the orientation set specified by etaMax
	bool cablePlatformPosistionCheck(const Vector3d& x);
	
	//! setup data structure to approximate the set of collision free platform positions
	void newCablePlatformRayBall();
	
	//! compute set of platform positions without cable-platform collisions around center
	void calculateCablePlatform();
	
	//! visualize collision cones
	void drawCablePlatformCones();
	
	//! visualize cable-plaftorm collision free position set 
	void drawCablePlatformCollisionFreeSpace();
	
	//! write collision cone data into .csv file
	bool writeCablePlatformConesToCSV(const string& filename);
	
	//! write triangulation indices of cable-platform data structure into .csv file
	bool writeCablePlatformTrianglelistToCSV(const string& filename);
	
	//! write ray vectors of cable-platform data structure into .csv file
	bool writeCablePlatformRaysToCSV(const string& filename);
	
	//! write platform geometry data into .csv file
	bool writeCablePlatformPlatformToCSV(const string& filename);


	// --------------------------- cable-print --------------------------- 

	//! draw printable superellipse connection between two layers
	void drawEllipseLayerConnection();

	//! visualize multiple layers of printable superellipses if the layers can be calculated independently 
	bool visualizeInterferenceCablePrintEllipseMultipleLayers();

	//! visualize multiple layers of printable parallelograms
	void visualizeInterferenceCablePrintParalellogramMultipleLayers();

	//! calculate multiple layers of printable parallelograms if the layers can be calculated independently
	bool calculateInterferenceCablePrintParalellogramMultipleLayers();
	
	//! visualize printable star-shaped polygon
	void visualizeInterferenceCablePrintStarShape();

	//! visualize printable superellipse
	void visualizeInterferenceCablePrintEllipse();

	//! visualize printable parallelogram
	void visualizeInterferenceCablePrintParallelogram();

	//! compute lists of characteristic and printable paralellograms 
	void calculateInterferenceWithPrintParallelogram();

	//! draw selected printable parallelogram from list
	void drawParallelogram();

	//! draw printable superellipse
	void drawEllipse();

	//! draw printable star-shaped polygon
	void drawStarShape();

	//! write star-shape data into .csv file
	bool writeStarShapeToCSV(const string filename);

	//! write parallelogram data into .csv file
	bool writeParallelogramToCSV(const string filename);

	//! write multiple layer parallelogram data into .csv file
	bool writeParallelogramMultipleLayerToCSV(const string filename);

	//! write superellipse data into .csv file
	bool writeEllipseToCSV(const string filename);

	//! callcula shadow centers and shadow scalings
	void calculatePrintShadowData();
    
	//! set vector of printing nozzle in the platform coordinate frame
	void setPn(Vector3d p);
	
	//! compute largest printable parallelogram with option to write data into .csv file
	bool calculateInterferenceWithPrintParallelogram(const string filename);
	
	//! compute largest printable superellipse
	void calculateInterferenceWithPrintEllipse();
	
	//! compute largest printable star-shaped polygon 
	void calculateInterferenceWithPrintStarShape();
	
	//! check if superellipses in the height interval [printcenter(2), printcenter(2)+printheight] can be calculated independently
	bool checkInterferenceWithPrintEllipseMultLayerCriteria();
	
	//! check if parallelograms in the height interval [printcenter(2), printcenter(2)+printheight] can be calculated independently
	bool checkInterferenceWithPrintParaMultLayerCriteria();


	// --------------------------- Old methods --------------------------- 

	explicit CInterference(CRobotData& robot);
	virtual ~CInterference(void);

	//! perform the calucations
	bool calculateInterference();

	//! perform the interference calculation for the orientation workspace
	bool calculateCollisionsOrientationWorkspace();

	//! provide the gloval interference region as matrix for use by other functiosn
	bool getInterferenceMatrix(MatrixXd& triangles);
	//! set the given value to the reference orientation matrix
	void setReferenceOrientation(Matrix3d& R) { R0=R; }
	//! set the orientation range for interference calculation
	void setOrientationWorkspace(const double& alpha_min, const double& alpha_max, const double& delta_alpha,
		const double& beta_min, const double& beta_max, const double& delta_beta,
		const double& gamma_min=0, const double& gamma_max=0, const double& delta_gamma=0);
	//! set the bounding box for outer clipping of the region
	void setClippingBox(double&xmin, double&xmax, double&ymin, double&ymax, double&zmin, double&zmax);
	//! get the orientation range for interference calculation
	void getOrientationWorkspace(double& alpha_min,double& alpha_max,double& delta_alpha,double& beta_min,double& beta_max,double& delta_beta) const;
	//! get the bounding box for outer clipping of the region
	void getClippingBox(double&xmin, double&xmax, double&ymin, double&ymax, double&zmin, double&zmax) const;

	// collision check between a triangulized obstacle and the cables of the robot
protected:
	//! data model for the obstacle
	vector<Vector3d> obstacle;
	//! experiment data model to stare a "collision free workspace"
	vector<Vector3d> validPositions;
public:

	// perform the calculation check between the cables and the obstacle
	bool calculateCollisions();

	// perform the calculation check between the cables and the obstacle for one pose (r,R)
	int getObstacleCableCollisions(const Vector3d& r, const Matrix3d& R);

	// load a STL object
	bool loadObstacle(const string& filename);

	// calculate collisions between cables for given pose
	bool calculateCableCableCollisions(const Vector3d& r, const Matrix3d& R, const bool& joints=false);
	// get collisions between cables for given pose
	void getCableCableCollisions(MatrixXd& collisions) {collisions=cableCableCollisions;}

	// calculate cable-cable collisions inside a box
	bool calculateCableCableBoxCollisions(const Vector3d& minBox, const Vector3d& maxBox, const Vector3d& orientation, const bool& joints=false);
	// get cable-cable collisions inside a box
	void getCableCableBoxCollisions(MatrixXd& collisions) {collisions=cableCableBoxCollisions;}

	// calculate collisions between cables and platform for given pose
	bool calculateCablePlatformCollisions(const Vector3d& r, const Matrix3d& R);
	// get collisions between cables and platform
	void getCablePlatformCollisions(MatrixXd& collisions) {collisions=cablePlatformCollisions;}

	// calculate cable-platform collisions inside a box
	bool calculateCablePlatformBoxCollisions(const Vector3d& minBox,const Vector3d& maxBox, const Vector3d& orientation);
	// get cable-platform collisions inside a box
	void getCablePlatformBoxCollisions(MatrixXd& collisions) {collisions=cablePlatformBoxCollisions;}
};

} // end namespace PCRL