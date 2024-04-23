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

/*! \file ForceLimits.h
 *
 *	\author   Andreas Pott
 *
 *  \dependency
 *		Eigen3		for IR³ algebra
 *
 *  The class CForceLimits implements a number of functions and algorithms
 *  to compute the cable force limits. 
 */

#pragma once

#include "Algorithm.h"

namespace PCRL {

/*! \class CForceLimits
 *  This class implements a collection of equations to compute the cable force limits.
 *  Parts of simplementation are related to the cable foce limits originally published
 *  In the respective ARK Paper. 
 *		Andreas Pott: On the Limitations on the Lower and Upper Tension for Cable-driven 
 *      Parallel Robots. In Advances in Robot Kinematics, p. 243-251, Springer, 2014. 
 */
class CForceLimits : public CAlgorithm
{
	double l,g;		//!< parameters for the cable model
protected:
	//! create the mapping between symbolic names and parameters
	virtual void bind() {}

	//! compute an improved estimate for the cable force H using Newton's method
	double NewtonStepSagging(double& H);

public:
	explicit CForceLimits(CRobotData& Robot);

	// put the implemented (or planned) functions here

	//! compute the minimum force required to reduce the sagging below smax for a horizontal cable with length l
	double getMinForceSag(const double& l, const double& g, double& smax, double H=1);
	//! compute the minimum force required to reduce the contraction of the cable below deltal for a horizontal cable with length l
	double getMinForceSaggingContraction(const double& l, const double& g,const double& deltaL, double H=1);
	//! compute the minimum force required to reduce the attack angle of the cable on the platform below deltaalpha for a horizontal cable with length l
	double getMinForceSaggingAttackAngle(const double& deltaalpha, const double& l);
	//! compute the minimum force required to achieve an eigenfrequncy for the cable of at least fe with length l
	double getMinForceEigenfrequency(const double& l, const double& g, const double& fe);
	
	//! compute the eigenfrequency of the cable for length l, weight g and tension H
	double getEigenfrequency(const double &l, const double& g, const double& H);
};

}	// end of namespace PCRL
