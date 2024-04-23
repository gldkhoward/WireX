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

/*!*******************************************************************
 *  \file   : NcDoc.cpp
 *
 *  Project   motionPlanning
 *
 *  \Author   Andreas Pott
 *
 *  \Date     20.05.2015
 *
 *  Last Change: 20.05.2015
 *
 *********************************************************************
 */ 

#include "NcDoc.h"
#include "NcParser.h"
#include "NcProgram.h"

namespace PCRL {

//! load the NC-program and generate the pose list
bool CNcDoc::loadNcProgram(const string& filename)
{
	//delete all poses from the last program (if any)
	PoseList.deleteAllPoses();
	
	if (filename.length()==0)
		return false;

	PCRL::CNcParser Parser=PCRL::CNcParser(); // initialize class with the nc-Parser
	PCRL::CNcProgram NcProgram=PCRL::CNcProgram(); // initialize a object, where the interpreted commands are saved

	cout << "Rufe Parser auf für " << filename << endl;

	// parse the program
	if (!Parser.ParseNcfile(filename.c_str(),NcProgram))
		return false; 

	if (NcProgram.empty())
	{	
		cout << "No Pose found, check file / filename" <<endl;
		return false;
	}

	// interpolate
	if (!Interpolator.interpolateProgram(NcProgram,PoseList))
		return false; 

	return true;
}


//! load the NC-program and generate the pose list
bool CNcDoc::parseNcProgram(const string& filename)
{
		
	if (filename.length()==0)
		return false;

	cout << "Call parser for " << filename << endl;
	
	NcProgram.clear();

	// parse the program
	if (!Parser.ParseNcfile(filename.c_str(),NcProgram))
		return false; 

	if (NcProgram.empty())
	{	
		cout << "No Pose found, check file / filename" <<endl;
		return false;
	}

	cout << "Successfully parsed the NC program." <<endl;
	return true;
}

//! load the NC-program and generate the pose list
bool CNcDoc::interpolateNcProgram()
{
	
	if (NcProgram.empty())
	{	
		cout << "No parsed NC program found." <<endl;
		return false;
	}

	// delete all poses from the last program (if any)
	PoseList.deleteAllPoses();

	// interpolate
	if (!Interpolator.interpolateProgram(NcProgram,PoseList))
		return false; 

	cout << "Successfully interpolated the NC program." <<endl;
	return true;
}

}	// namespace PCRL 