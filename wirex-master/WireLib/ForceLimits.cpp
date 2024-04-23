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
 *  \file   : ForceLimits.cpp
 *
 *  Project   Wirelib
 *
 *  \Author   Andreas Pott
 *
 *  \Date     16.01.2014
 *
 *  Last Change: 16.01.2014
 *
 *********************************************************************
 */ 

#include "ForceLimits.h"
#include <Eigen/Core>


namespace PCRL {

CForceLimits::CForceLimits(CRobotData& Robot) : CAlgorithm(Robot)
{
}

//! compute the minimum force required to reduce the sagging below smax 
//! for a horizontal cable with length l
double CForceLimits::getMinForceSag(const double& l, const double& g, double& smax, double H)
{
	const double eps = 1e-6;		// the lower bound for the force H; if the value computed from
							// newtons method is lower, than we set it the eps to avoid
							// phically unfeasbile values.
	
	// we blindly perform 20 Newton Steps on the sagging equation and hope that the solution is good enough
	for (int i=0; i<20; i++)
//		H = -g*H*(sinh(g*l/H/2.0)*l-2.0*smax)/(2.0*H*cosh(g*l/H/2.0)-2.0*H-sinh(g*l/H/2.0)*l*g);
		// we know that we are searching for a positive force and forces lower than 1e-6
		// seem to be irrelevant. furthermore, we expect the function to be monotone
		H = max(eps,-g*H*(sinh(g*l/H/2.0)*l-2.0*smax)/(2.0*H*cosh(g*l/H/2.0)-2.0*H-sinh(g*l/H/2.0)*l*g));

	return H;
}

//! compute the minimum force required to reduce the contraction of the cable below deltal for a horizontal cable with length l
double CForceLimits::getMinForceSaggingContraction(const double& l, const double& g,const double& deltaL, double H)
{
	const double eps = 1e-6;// the lower bound for the force H; if the value computed from
							// Newtons method is lower, than we set it the eps to avoid
							// phically unfeasbile values.	for (int i=0; i<20; i++)
	// we blindly perform 20 Newton steps on the sagging equation and hope that the solution is good enough
	double dL = deltaL+l;
	for (int i=0; i<20; i++)
		H = -H*g*(cosh(g*l/H/2.0)*l-dL)/(-cosh(g*l/H/2.0)*l*g+2.0*sinh(g*l/H/2.0)*H);
	return H;
}

//! compute the minimum force required to achieve an eigenfrequncy for the 
//! cable of at least fe with length l and weight per length g
double CForceLimits::getMinForceEigenfrequency(const double& l, const double& g, const double& fe)
{
	// the eigenfrequency is computed from physic folklore ...
	return 4*fe*fe*l*l*g;
}

//! compute the eigenfrequencies with a simple formula and return the result
double CForceLimits::getEigenfrequency(const double &l, const double& g, const double& H)
{ 
	return sqrt(H/g)/l/2.0; 
}

} // end namespace PCRL