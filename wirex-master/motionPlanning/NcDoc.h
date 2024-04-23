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

/*! \file NcDoc.h
 *
 *	\author   Andreas Pott
 *
 *  \brief
 *  This is a helper class to provide a basic data model to operate on.
 *  The implementation provide the playgroup to perform some operations with 
 *  the interpolator and a pose list by aggretating instances of these classes.
 */

#pragma once
#include <motionPlanning/NcInterpolator.h>
#include <motionPlanning/NcParser.h>
#include <WireLib/PoseList.h>

namespace PCRL {

class CNcDoc
{
public:
	CNcParser Parser; // initialize class with the nc-Parser
	CNcProgram NcProgram; // initialize a object, where the interpreted commands are saved
	
	CNcInterpolator Interpolator;
	CPoseListKinetostatic PoseList;

	//! load the NC-program and generate the pose list
	bool loadNcProgram(const string& filename);
	//! parse a NC-program
	bool parseNcProgram(const string& filename);
	//! interpolate a NC-program and generate the pose list
	bool interpolateNcProgram();
};

} // end of namespace PCRL
