/*! \file RobotData6.h                                                   
 *
 *	\author   Andreas Pott
 *
 *  (C)opyright 2009-2016, Andreas Pott
 *  Fraunhofer IPA
 *
 * This class is deprecated and will be removed in a future version.
 */

#pragma once

#ifdef USE_DEPRECATED_ROBOTDATA6_CLASS

#include "RobotData.h"
#include "EigenLib.h"

namespace PCRL {

/*! \class CRobotData6
 *  This class extends the pure geometry and technology of the wire robot
 *  with a state-based behaviour for a spatial redundent wire robot.
 *  The class is intended to simplify interaction by supplying a more specialized
 *  robot class.
 *  The class stores a kinematic states (r,R) as well as the wire length l
 *  in internal variables. Futhermore, it provides a collection of algorithm 
 *  for statics, kinematics, interference, etc.
 *
 *  \remark This class is deprecated and will be remove in a future version of WireLib.
 */
class CRobotData6 : public CRobotData
{
public:
	Vector3d r;				//!< the actual position of the robot
	Matrix3d R;		//!< the actual orientation of the robot
	double *l;				//!< the actual length of the wires
	double stepsize;		//!< max distance between to samples of the interpolator

	// algorithm object to provide some "interesting" calculations
//	CWorkspaceHull WorkspaceHull;
//	CWorkspaceCrosssection Crosssection;
//	CKinemtics Kinematics;
//	CInterference Interference;

public:
private: // this private statement prevents the usage of this class since the current state of the implementation is not ready
	CRobotData6(const int& NoW);
	virtual ~CRobotData6();

	//! calculate the kinematics
	bool doForwardKinematics();
	bool doInverseKinematics();
};

} // namespace PCRL

#endif //USE_DEPRECATED_ROBOTDATA6_CLASS
