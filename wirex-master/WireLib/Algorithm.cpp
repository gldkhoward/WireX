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
 *  \file   : Algorithm.cpp
 *
 *  Project   Wirelib
 *
 *  \Author   Andreas Pott
 *
 *  \Date     15.12.2009
 *
 *  Last Change: 30.07.2012
 *
 *********************************************************************
 */

#include "Algorithm.h"

namespace PCRL {

CAlgorithm::CAlgorithm(CRobotData& Robot)
{
	pRobot = &Robot;
	pReflector = 0;
}

//! during destruction we have to delete the reflector (if needed)
CAlgorithm::~CAlgorithm(void)
{
	if (pReflector)
		delete pReflector;
}

/*! there is nothing that we have to bind in the base class
 *  anyway, we have to provide a default implementation.
 *  when overloading this functions, do not forget to call
 *  the function of the parent class!
 */
void CAlgorithm::bind()
{
}

/*! return a reference to the reflector object for this class.
 *  the reflector object allows for introspection, parameter
 *  persistency, and set/get by symbolic name.
 *  If no such object exists it is create once the first time it
 *  is needed and then stored for later use. If the use of the
 *  reflector is time critical, make a dummy call e.g.
 *    alg_obj.Reflector();
 *  at an early phase where some very small delay for building the
 *  binding tables is acceptable.
 */
CReflection& CAlgorithm::Reflector()
{
	// if we no not have a reflector we create one
	if (!pReflector)
	{
		pReflector = new CReflection;
		// let the virtual function bind() fill the binding table
		bind();
	}
	// now we can return the reflector
	return *pReflector;
}

} // end namespace PCRL