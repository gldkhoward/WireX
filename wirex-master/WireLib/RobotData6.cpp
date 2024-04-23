/*!*******************************************************************
 *  \file   : RobotData6.cpp
 *
 *  Project   Wirelib
 *
 *  \Author   Andreas Pott
 *
 *  \Date     10.10.2008 (part of MoWireRobot6)
 *			  15.12.2009 (refactored by asp)
 *
 *  (c)       Fraunhofer IPA
 *
 *********************************************************************
 */ 

#include "RobotData6.h"

namespace PCRL {

#ifdef USE_DEPRECATED_ROBOTDATA6_CLASS

CRobotData6::CRobotData6(const int& NoW) : CRobotData(NoW,MP3R3T)
{
	l = new double[getNow()];
	r.setZero();
	R = Matrix3d::ZRotationMatrix3d(0);
	stepsize = 0.02;
}

CRobotData6::~CRobotData6()
{
	delete [] l;
}

//! wrapper for forward kinematics; use the class members to hold pose and 
//! length of wires
bool CRobotData6::doForwardKinematics()
{
	return false; // use an algorithm object here!
}

//! wrapper for inverse kinematics; use the class members to hold pose and 
//! length of wires
bool CRobotData6::doInverseKinematics()
{
	return false; // use an algorithm object here!
}

#endif //USE_DEPRECATED_ROBOTDATA6_CLASS

} // namespace PCRL