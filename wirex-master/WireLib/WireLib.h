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

/*! \mainpage WireLib Documentation

    \section general General
	This is the code documentation of the WireX project, 
	(C)opyright 2009-2018 &copy; Andreas Pott, Fraunhofer IPA

	Wirelib is a class library with data and algorithms classes to analyse and 
	design parallel cable robots. All entities are encapsulated in the 
	namespace PCRL (parallel cable robot library).
	
    The package containts classes representing
	- data object to descibe cable-driven parallel robots 
	- algorithm objects for kinematic transformation, structure matrix, 
	  force distributions, stiffness, interference, and other effects
     
	\section WireCenter
	WireCenter is a GUI for making use of the functions and classes in WireLib.
	It is only intended for internal use for present and future developers. 
	WireCenter is implemented in Visual Studio 2010 and clustered into the following libraries:\n
		- WireCenter
		- WireLib
		- IPAGL
		- motionPlaningLib
		- Trafo_DLL (standalone - Visual Studio 6)

	Using the following third party libaries:
		- Eigen3
		- tinyXML
		- levmar
		- sqlite3
	
	\section Supplement Supplement libraries
	The are currently to supplements to wirelib. Firstly, Shapes.h provides some
	handy classes for visualization of wirelib's objects in openGL.
	Secondly, WirelibPyBindings gives access to many wirelib function through python.

	\section Convenstions Descriptive Conventions for Developers
	Header Files should contain classes and their description, 
	and brief descriptions of member functions/variables. 
	hereas a more detailed description should be in the .cpp 
	files, specifying input parameters (\\param) and return 
	values (\\return).
	
	Eg: Brief Description in Header File:\n
	////! evaluate the inverse kinematics and compare the results with 
	the minimum and maximum cable length\n
	bool testWorkspaceWireLength(const Vector3d& r, const Matrix3d& R);\n
	-or-\n
	MatrixXd k_spec; //!< local array of spring constants (for performance reasons)
	
	Eg: Detailed Description in .cpp File:\n
	\code
	//! Calculate the inverse velocity transmission of the robot based on the 
	//! simplified kinematic model. The algorithms is based on the equations
	//! given in R. Verhoeven (2004): "Analysis of the Workspace of Tendon-based 	
	//! Stewart Platforms", p.99. This function basically evaluates the the 
	//! product dl=A(r,R)*[v, omega], where A is the pose dependent structure
	//! matrix (not transposed!), [v,omega] is the twist of the platform and (r,R)
	//! is the pose of the platform.
	//! 
	//! \param r [in] is the position of the platform
	//! \param R [in] is the orientation of the platform as rotation matrix
	//! \param v [in] is the linear velocity of the platform
	//! \param omega [in] is the angular velocity of the platform
	//! \param dl [out] is the pointer to an array with now elements that contains the desired cable velocities
	//! \return true, if successful; if the function return false, the value of dl is undefined.
	//!	//! \remark The current implementation is rather preliminary, since it makes 
	//! no use of the position inverse kinematics. Furthermore, the kinematic code
	//! was not optimize w.r.t performance. 
	bool CKinematics::doInverseKinematicsVelocity(const Vector3d&r, const Matrix3d& R, const Vector3d& v, const Vector3d& omega, double* dl)
	\endcode
	\n

    \section Literature
    A short review on related literatur (both algorithm and application)
    Further references can be found in the documentation in doc/WireDoc.tex that also
	belongs to this repo.

	A developers guide is available under doc/WireDoc.tex including WireCenter's users manual
	and hints and plans for the development of this software package. This documentation also
	includes in specification of the XML based file format used in WireLib.

    \section Revision History 
	  - Version 0.3 "Ursa-Minor": valid til 5th Octobre 2011; uses mobile and MoAlgebra
	  - Version 0.4 "Sagittarius": valid since 6th Octobre 2011; linear algebra subsystem was ported to eigen3
	  - Version 1.0 "Sirius": valid since 13th Decembre 2012; first release version
	  - Version 1.1 "Sirius": valid since 17.10.2014 after major updated and extensions in WireCenter with improved GUI
	  - Version 2.0 "Fornax": valid since 03.02.2016. Conversion to English GUI, split of motionPlaningLib  
	  - Version 2.1 "Sculptor": valid since 03.04.2017. Collection of GUI patches
	  - Version 3.0 "Centaurus": valid since 29.05.2019. relauch as open source project

    \namespace PCRL
    The namespace PCRL (parallel cable robot library) is an algorithm and data 
	structure collection for analysis and synthesis of cable-driven parallel 
	robots.	  
*/

#pragma once

#include "RobotData.h"

namespace PCRL {

	static const char *majorVersion="3";				// manually set
	static const char *minorVersion="0";				// manually set
	static const char *buildVersion="1022";				// hand-coded from git;
	static const char *versionString="3.0.1022";		// manually composed the three lines above
	static const char *versionName="Centaurus";			// manually set as defined by the maintainer
	static const char *name="WireLib -- Parallel Cable Robot Library";

} // namespace PCRL
