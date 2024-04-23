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

// pyBindings.cpp: Implementierungsdatei
//

#include "pyBindings.h"
using namespace PCRL;

CNcDoc* pNcDoc = NULL;

PCRL::CNcDoc* getDoc()
{
	return pNcDoc;
}

/* create a default robot document object */
static PyObject *createInterpolator(PyObject *self, PyObject *args)
{
	if (pNcDoc)
		delete pNcDoc;
	// create a eight wire, fully-constrained spatial default robot with generic geometry
	pNcDoc = new CNcDoc();
    return Py_BuildValue("i", 55);
}

/* Delete the current robot. Further calls to robot related function will fail
 */
static PyObject *deleteInterpolator(PyObject *self, PyObject *args)
{
	if (pNcDoc) 
		delete pNcDoc;
	pNcDoc = 0;
	Py_RETURN_TRUE;
}

/*! return the version string defined by wirelib
 */
static PyObject *version(PyObject *self, PyObject *args)
{
	//! \todo we might refer to a constant static variable in motionPlanning once such a version is provided...
	//!		  until then, we return a constant here.
	return Py_BuildValue("s","1.0");
}

PyMethodDef WireLibMethods[] = {
    {"version", version, METH_NOARGS, "return the version information of WiPy as string"},
	{"createRobot", createInterpolator, METH_NOARGS, "Create new robot. You must call this function to initalize a robot prior to use."},
	{"deleteRobot", deleteInterpolator, METH_NOARGS, "Destroy the current robot object."},
    {NULL, NULL, 0, NULL}  
};
