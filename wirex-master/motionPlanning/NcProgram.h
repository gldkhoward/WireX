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

/*! \file NcProgram.h
 *
 *	\author   Andreas Pott, Werner Kraus
 *
 *  Implement a data model to store a parsed G-code NC program
 */

#pragma once

#include <list>
#include <string>
#include <WireLib/EigenLib.h>

namespace PCRL {

/*! abstract base class for different types of commands
 */
class CNcCommand
{
	virtual void printContent() {}
};

//! a simple linear motion
class CNcLinear : public CNcCommand
{
public:
	// data model
	Vector3d targetposition;	//!< the targeted position of the linear motion
	Vector3d targetorientation;	//!< the targeted orientation of the linear motion
	double v;		//!< the programed velocity of the command
	void printContent(){;}
};

//! pause the movement for a defined duration
class CNcDwellTime : public CNcCommand
{
public:
	// data model
	Vector3d targetposition;	//!< the targeted position of the linear motion
	Vector3d targetorientation;	//!< the targeted orientation of the linear motion
	double time;		//!< dwelltime in seconds
};


//! a simple circular motion
class CNcCircular : public CNcCommand
{
	void printContent(){;}
};


//! a comment
class CNcComment : public CNcCommand
{
public:
	// data model
	std::string comment;	//!< a comment
	void printContent(){;}
};

//! M-command
class CNcMcommand : public CNcCommand
{
public:
	// data model
	int commandNr;	//!< Nr of the M-command
	void printContent(){;}
};

//! we can define any number of additional NC-commands to extend the behaviour
//! of NC programs

/*! a NC program consisting of a list of CNcCommand objects
 *  the single commands are passed to 
 *  the program and CNcProgram takes care of correctly
 *  freeing the memory where the program is disposed. This data structure
 *  is generated from the NcParser and can be used as input for the
 *  NcInterpolator. 
 */
class CNcProgram : public std::list<CNcCommand*>
{
public:
	CNcProgram();
	//! free the memory of the dynamically allocated objects and then empty the list
	~CNcProgram();
};



} // end of namespace PCRL
