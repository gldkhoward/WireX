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

/*! \file Algorithm.h
 *
 *	\author   Andreas Pott
 *
 *  \dependency
 *		Eigen3		for IR³ algebra
 *		TinyXml		for XML support
 *
 *  Base class for the implementation of a library of algorithms for 
 *  cable-driven parallel robots. CAlgorithms does nothing itself but
 *  provides the definitions and infrastructure for actual algorithm
 *  classes. Currently, two main functions are implemented to be shared
 *  amongst all algorithm objects: firstly a pointer of the connected
 *  robot instance. This object must be passed during initialization of
 *  the algorithms. Secondly, an internal object is prepared to allow
 *  for reflection, i.e. functions allow to inspect the classes variables 
 *  at runtime, given that these variables are registered using the virtual
 *  bind() method. The bind() method maps member variables to both
 *  a symbol name (stored as a string) and a xpath-compatible adress to be
 *  loaded and saved from an XML file.
 *  
 *  When implementing custom classes, all member variables representing the
 *  configuration settings shall be registed by an overridden bind() function.
 *  Be careful only to register variables with a static behaviour.
 */

#pragma once

#include <string>
#include <vector>
#include "RobotData.h"
#include "Reflection.h"

namespace PCRL {

/*! Data structure to assembly one surface triangle of a workspace
 *  from the index of the vertices.
 */
struct CTriangleIndices
{
	int i,j,k;
	CTriangleIndices() : i(0),j(0),k(0) {}
	CTriangleIndices(int I, int J, int K) : i(I),j(J),k(K) {}
};

/*! \class CAlgorithm
 *  Abstract base class for cable robot algorithms
 */
class CAlgorithm
{
protected:
	//! implement the parameter binding in this class
	CReflection *pReflector;
	//! create the mapping between symbolic names and parameters
	virtual void bind();
	//! robot model (geometry, technological parameters)
	CRobotData* pRobot;
public:
	explicit CAlgorithm(CRobotData& Robot);
	~CAlgorithm(void);
	//! return a reflector object for this instance to be used for introspection, generic getter/setter as well as generic parametrer serialization in text and xml files
	CReflection& Reflector();
};

/*! CPoseProperty is the virtual base class for all algorithm that perform 
 *  pose dependent calculations, such as computing kinematic codes, setting up
 *  the structure matrix, dexterity, stiffness, singularities, force 
 *  distributions. Childs of pose property represent pose depend and thus local
 *  characteristics of the robot. 
 *  The interface of CPoseProperty is used by other algorihtms (such as 
 *  workspace determination) to call testing functions through the virtual 
 *  function(s) of CPoseProperty without caring for the technical details of 
 *  the evalations and configuration of the called objects.
 * 
 *  Furthermore, there is an virtual interface to compute pose dependent properties
 *  and return the values in a standarized way as a vector of performance indicators
 *  (whatever may be the meaning of performance indicators). The interface allows for
 *  iterating through containers of PoseProperty and compute the specific data
 *  for each property in a unified way. 
 */
class CPoseProperty : public CAlgorithm
{
public:
	explicit CPoseProperty(CRobotData& Robot) : CAlgorithm(Robot) {}
	~CPoseProperty() {}

	//! test if some specific property is fulfilled for a given pose (r,R) 
	virtual bool testPose(const Vector3d& r, const Matrix3d& R) { return false; }
	//! get the number of properties computed
	virtual int getPropertyCount() { return 0; }
	//! return the names of the properties to be used as headers in tables by appending the names to the vector
	virtual bool getPropertyNames(std::vector<string>& names) { return false; }
	//! do the actual computation of the properties for the given pose (r,R)
	virtual bool computeProperty(const Vector3d& r, const Matrix3d& R, MatrixXd& values)  { return false; }
};

class CPoseDynProperty : public CPoseProperty
{
public:
	explicit CPoseDynProperty(CRobotData& Robot) : CPoseProperty(Robot) {}
	~CPoseDynProperty() {}

	//! test if some specific property is fulfilled for a given pose (r,R) 
	virtual bool testPose(const Vector3d& r, const Matrix3d& R, const Vector3d& v,
					  const Vector3d& omega, const Vector3d& a,const Vector3d& alpha,
					  const Vector3d& f, const Vector3d& tau) { return false; }
	//! get the number of properties computed
	//virtual int getPropertyCount() { return 0; }
	//! return the names of the properties to be used as headers in tables by appending the names to the vector
	//virtual bool getPropertyNames(std::vector<string>& names) { return false; }
	//! do the actual computation of the properties for the given pose (r,R)
	virtual bool computeProperty(const Vector3d& r, const Matrix3d& R, const Vector3d& v,
					  const Vector3d& omega, const Vector3d& a,const Vector3d& alpha,
					  const Vector3d& f, const Vector3d& tau, MatrixXd& values)  { return false; }
};


}	// end of namespace PCRL
