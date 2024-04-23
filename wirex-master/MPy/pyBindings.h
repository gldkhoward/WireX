/*
* WireX  -  MPy
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

/*! \file pyBindings.cpp
 *
 *	\author   Philipp Miermeitser
 *
 *  \dependency
 *		Python		scripting interface
 *		WireLib		function library to be interfaced
 *
 *  \brief
 *  Interface definition for the exported modules in WiPy. In pyBindings.*
 *  the interface table of the module WiPy is implemented. This includes
 *  the services that are specific to the stand alone WiPy module and that
 *  are not present in the WireCenter GUI version. 
 */

#pragma once

#ifndef WIN32
#include <python2.7/Python.h>
#else
#include <Python/Python.h>
#endif

#include <motionPlanning/NcDoc.h>


using namespace PCRL;
extern CNcDoc* pNcDoc;

// Python helper functions
extern PyMethodDef WireLibMethods[];


