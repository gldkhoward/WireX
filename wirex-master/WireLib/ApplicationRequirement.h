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

/*! \file ApplicationRequirement.h
 *
 *	\author   Andreas Pott
 *
 *  \dependency
 *		Eigen3		for IR³ algebra
 *
 *  Definition of application related requirements for a cable robot
 */

#pragma once

#include <string>
#include"EigenLib.h"

namespace PCRL {

/*! \class CApplicationRequirement
 *  Data structure to describe the requirements of a certain application in terms
 *  of workspace, payload, and dynamics
 */
class CApplicationRequirement
{
public:
	Vector3d minWS;		//!< minimum corner of the bounding box of required workspace
	Vector3d maxWS;		//!< maximum corner of the bounding box of required workspace
	Vector3d minIS;		//!< minimum corner of the bounding box of the installation space
	Vector3d maxIS;		//!< minimum corner of the bounding box of the installation space


	double velocity;	//!< maximum velocity of the mobile platform of the robot
	double acceleration;//!< maximum acceleration of the mobile platform of the robot
	double payload;		//!< payload of the robot system [N]
	double force;		//!< maximum applied force
	double torque;		//!< maximum applied torque

public:
	CApplicationRequirement(void);
	~CApplicationRequirement(void);

	//! store the requirement settings in an xml document
	bool loadXML(const string& filename);
	//! load the requirement settings in an xml document
	bool saveXML(const string& filename);
};

}	// end of namespace PCRL