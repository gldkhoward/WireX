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

/*! \file NcParser.h
 *
 *	\author   Andreas Pott, Werner Kraus
 *
 *  The class CNcParser interpretes a NC-program written in G-Code and saves the Nc-Commands
 *  in a list for further processing like interpolation
 */


#pragma once

#include <map>
#include <string>
#include <motionPlanning/NcInterpolator.h>
#include <WireLib/EigenLib.h>

#pragma warning(disable:4091)

namespace PCRL {

class CNcParser
{
public:
	CNcParser(void);
	~CNcParser(void);
	//! parse a NC file using G-Code and create the a list of NC commands
	bool ParseNcfile(const char* filepath, CNcProgram& Program);
	//! exports the target positions of a parsed Nc program to a text file 
	bool exportNcProgram(CNcProgram& Program);

	//! reads a double value from a string 
	bool readDouble(string& sInput, int& iStart, int& iEnd, double& dValue);
	//! reads an integer value from a string
	bool readInteger(string& sInput, int& iStart, int& iEnd, int& iValue);
	//! specific parser function to read nc syntax and get the values
	bool readValue(string& sInput, int& iStart, int& iEnd, double& dValue);
	//! help function to trim a string
	void trim(string& str);

	
public:
	typedef  enum  handling{NC_SAVE, NC_IGNORE, NC_PRINT, NC_SAVE_PRINT}; // unused: the idea is to define if a syntax is either saved to the list, ignored, printed, or aved and printed
	handling eGCommand;
	handling eMCommand;
	handling eSharpCommand;
	handling eComment;


	typedef  enum  MotionMode{MOTION_MODE_SEEK, MOTION_MODE_LINEAR, MOTION_MODE_CW_ARC, MOTION_MODE_CCW_ARC, MOTION_MODE_DWELL}; // different types of NC commands
	MotionMode eMotionMode;
	typedef  enum  PositionMode{POSITION_ABSOLUTE, POSITION_RELATIVE}; // G90 G91
	PositionMode ePositionMode;

	std::map<string,int> mAxis; // relation between axis-name in G-Code and the number of the axis. E.G. W1=0

	std::map<int,double> mVariables; // relation between variables defined in G-Code like P10
	//! set the handling of commands
	void setHandling(handling eGCommand, handling eMCommand, handling eSharpCommand,handling eComment);
	//! set the axis relation between axis-name in G-Code and the number of the axis. E.G. W1=0
	void setAxisMap(int axis, string cName);
};

} // end of namespace PCRL

