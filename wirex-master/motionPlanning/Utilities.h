/*
* WireX  -  motionPlanning
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

/*! \file Utilities.h
 *
 *	\author   Philipp Miermeister
 *
 *  \dependency
 *		Eigen3		for IR³ algebra
 *      tinyXML		for XML-document parsing
 * 
 *  \par brief
 *  Implementation of a collection of algorithms for parallel cable robots. 
 */

#pragma once

#include <WireLib/EigenLib.h>
#include <tinyXML/tinyxml.h>
#include <assert.h>
#include <time.h>
#include <WireLib/EnumTables.h>


namespace PCRL {
	
/*! \class CRay
 * A ray is defined by its origin o and the direction d
 */
class CRay {
  public:
	CRay();
	CRay(const Vector3d& o, const Vector3d& d);
	CRay(const CRay &r);

	Vector3d origin;
	Vector3d direction;
	Vector3d inv_direction;
	int sign[3];
};

/*! \class CBox
 *  Axis aligned bounding box
 */
class CBox {
  public:
	CBox();
	CBox(const Vector3d &min, const Vector3d &max);
	// (t0, t1) is the interval for valid hits
	//bool intersect(const Ray &, float t0, float t1) const;
	bool intersect(const CRay &r) const;

	//! return a reference to the lower corner
	Vector3d& lower() { return parameters[0]; }
	//! return a reference to the upper corner
	Vector3d& upper() { return parameters[1]; }
	//! normalize the box
	void normalize() { for (int i=0; i<3; i++) { if (parameters[0](i) > parameters[1](i)) swap(parameters[0](i),parameters[1](i)); }}
	
	//! corners
	Vector3d parameters[2];
};

/*! Edge structure (is used for the generation process)
 */
class CEdge {
public:
	unsigned int v1ID;
	unsigned int v2ID;
	int ed1ID;
	int ed2ID;
	bool inverse;

	CEdge(unsigned int v1ID, unsigned int v2ID, int ed1ID, int ed2ID);	
	void invert();
}; 

// some further utility functions
//! determine the time to perform a one-dimensional point-to-point trajectory taking into account 
//! path length s, maximum velocity vmax, maximum acceleration amax, and maximum yerk rmax. All
//! parameters have to be passed in (meter, second) unit system
double getTrajectoryTime(const double& s, const double& vmax, const double& amax, const double& rmax);

// XML helper functions

//! read the position vector attributes of a given tag "parent" and store it in value
bool getPositionVectorAttrib(const TiXmlElement* parent, Vector3d& value);

//! write the position vector attributes into a given tag "parent" 
bool setPositionVectorAttrib(TiXmlElement* parent, const Vector3d& value);

//! get the orientation matrix attributes of a given tag "parent" and store it in value
bool getOrientationMatrixAttrib(const TiXmlElement* parent, Matrix3d& value);

//! get the inertia tensor matrix attributes of a given tag "parent" and store it in value
bool getInertiaTensorAttrib(const TiXmlElement* parent, Matrix3d& value);

//! get the quaternion attributes of a given tag "parent" and store it in value
bool getQuaternionAttrib(const TiXmlElement* parent, double& q0, double& q1, double& q2, double& q3);

//! get the axis angle attributes of a given tag "parent" and store it in value
bool getAxisAngleAttrib(const TiXmlElement* parent, double& beta, double& ux, double& uy, double& uz);

//! write the orientation matrix attributes into a given tag "parent" 
bool setOrientationMatrixAttrib(TiXmlElement* parent, const Matrix3d& value);

//! write the Tensor Matrix attributes into a given tag "parent"
bool setInertiaTensorAttrib(TiXmlElement* parent, const Matrix3d& value);

//! read a scalar attribute of a given tag "parent" and store it in value
bool getScalarAttrib(const TiXmlElement* parent, double& value);

//! get the xyz vector from Rotation Matrix
bool getXYZFromMatrix(Vector3d& abc, const Matrix3d& R);

//! get the XYZ from Rotation Matrix
bool getXYZFromMatrix(double& a, double& b, double& c, const Matrix3d&R);  

//! get the rotation matrix from a quaternion
bool getMatrixFromQuaternion(Matrix3d& R, const double& q0, const double& q1, const double& q2, const double& q3);

//! get rotation matrix form euler angels
bool getMatrixFromXYZ(Matrix3d& R, const Vector3d& abc);

//! compute a uniform distributed random orientation
bool getMatrixRandomOrientation(Matrix3d& R);

//! get the quaternion from a rotation matrix
bool getQuaternionFromMatrix(double& q0, double& q1, double& q2, double& q3, const Matrix3d& R);

//! get Axis Angle from rotation matrix
bool getAxisAngleFromMatrix(double& beta, double& ux, double& uy, double& uz, const Matrix3d& R);

//! get rotation matrix from Axis Angle
bool getMatrixFromAxisAngle(Matrix3d& R, const double& beta, const double& ux, const double& uy, const double& uz);

//! get Axis Angle from quaternion
bool getAxisangleFromQuaternion(double& beta, double& ux, double& uy, double& uz, const double& q0, const double& q1, const double& q2, const double& q3);

//!get quaternion from Axis Angle
bool getQuaternionFromAxisAngle(double& q0, double& q1, double& q2, double& q3, const double& beta, const double& ux, const double& uy, const double& uz);

//! get Rodrigues Parameters from Axis Angle
bool getRodriguesFromAxisAngle(double& Rx, double& Ry, double& Rz, const double& beta, const double& ux, const double& uy, const double& uz);

//! get Axis Angle from Rodrigues
bool getAxisAngleFromRodrigues(double& beta, double& ux, double& uy, double& uz, const double& Rx,const double& Ry,const double& Rz);


//! get Distance between two Rotation Matrices in SO_3
double getSO3Distance(const Matrix3d& R_1, const Matrix3d& R_2);

// collection of geometric algorithms
/////////////////////////////////////

//! check for intersection between a triangle and a line segment
bool getIntersectionTriangleLine(const Vector3d& a, const Vector3d& n_a, const Vector3d& u, const Vector3d& n_v, const Vector3d& n_w);

bool getEigenvectorBase(const Matrix3d& K, Matrix3d& eigenvectors, Vector3d& eigenvalues);

//! adds errors within a given range on a given set of cablelenths
void generateWireErrors(MatrixXd& l, int now, MatrixXd& wireErrors, emWireErrors::Type wireErrorType, double errorrange);

//! Get current date/time, format is YYYY-MM-DD HH:mm:ss
const string currentDateTime();

//! Get Standard Deviation
double stddev(const MatrixXd& v);

bool Gauss_Newton(void (*func)(double*p, double *l,int m, int n,void *adata), double*p,int itmax,double *info,int m, int n,double del,void *adata);

} // namespace PCRL