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

/*! \file WiPy.cpp
 *
 *	\author   Philipp Miermeitser
 *
 *  \dependency
 *		Python		scripting interface
 *		WireLib		function library to be interfaced
 *      WireLibPyBindings
 *					the method table with adapter functions converting function calls python to wirelib
 *
 *  \brief
 *  WiPy is the main file to build a stand-alone python interface library called WiPy
 *  allowing to use parts of wirelib in python.
 */	

#include "pyBindings.h"
#include <motionPlanningPyBindings/motionPlanningPyBindings.h>

// using namespace PCRL;

// important: add all python moudles that should be export under Properties->Linker->Command Line->Additional Options
// as /export:initMPy

PyMODINIT_FUNC
initMPy(void)
{
    (void) Py_InitModule3("MPy", WireLibMethods, "MPy is a stand alone python module for motion planning");
	(void) Py_InitModule3("Icontrol", EmbMethodsControl, "Icontrol has functions to load and process path data loaded from nc files.");
	PyRun_SimpleString("import Icontrol");
}


int main(int argc, char *argv[])
{
    /* Pass argv[0] to the Python interpreter */
    Py_SetProgramName(argv[0]);

    /* Initialize the Python interpreter.  Required. */
    Py_Initialize();

    /* Add a static module */
    initMPy();
}

