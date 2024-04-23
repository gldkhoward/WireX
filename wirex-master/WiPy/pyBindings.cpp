/*
 * WireX  -  WiPy
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

#include <WireLib/WireLib.h>
#include "pyBindings.h"
using namespace PCRL;

CRobotDocument* pRobotDoc = NULL;

PCRL::CRobotDocument* getRobotDocument()
{
	return pRobotDoc;
}

/* This setup the basic robots and thus defines the defaults robots properties 
*/
void createRobotCore()
{
	// create a eight wire, fully-constrained spatial default robot with generic geometry
	if (pRobotDoc)
		delete pRobotDoc;
	pRobotDoc = new CRobotDocument(8,CRobotData::MP3R3T);
	pRobotDoc->setBase(0,Vector3d(-1,-1,0));
	pRobotDoc->setBase(1,Vector3d( 1,-1,0));
	pRobotDoc->setBase(2,Vector3d( 1, 1,0));
	pRobotDoc->setBase(3,Vector3d(-1, 1,0));
	pRobotDoc->setBase(4,Vector3d(-1,-1,2));
	pRobotDoc->setBase(5,Vector3d( 1,-1,2));
	pRobotDoc->setBase(6,Vector3d( 1, 1,2));
	pRobotDoc->setBase(7,Vector3d(-1, 1,2));
}

/* create a default robot document object */
static PyObject *createRobot(PyObject *self, PyObject *args)
{
	createRobotCore();
    return Py_BuildValue("i", 55);
}

/* Delete the current robot. Further calls to robot related function will fail
 */
static PyObject *deleteRobot(PyObject *self, PyObject *args)
{
	if (pRobotDoc) delete pRobotDoc;
	pRobotDoc=0;
	Py_RETURN_TRUE;
}

static PyObject *loadRobot(PyObject *self, PyObject *args)
{
	std::string sFileName = "IPAnema.txt";
	const char* filename = sFileName.c_str();
	if ( pRobotDoc->Load(filename) ) {
        Py_RETURN_TRUE;
    }

    Py_RETURN_FALSE;
}

/*! return the version string defined by wirelib
 */
static PyObject *version(PyObject *self, PyObject *args)
{
	return Py_BuildValue("s",PCRL::versionString);
}

/*! return the major version string defined by wirelib
 */
static PyObject *versionMajor(PyObject *self, PyObject *args)
{
	return Py_BuildValue("s",PCRL::majorVersion);
}

/*! return the minor version string defined by wirelib
 */
static PyObject *versionMinor(PyObject *self, PyObject *args)
{
	return Py_BuildValue("s",PCRL::minorVersion);
}

/*! return the build version string defined by wirelib
 */
static PyObject *versionBuild(PyObject *self, PyObject *args)
{
	return Py_BuildValue("s",PCRL::buildVersion);
}

PyMethodDef WireLibMethods[] = {
    {"version", version, METH_NOARGS, "return the version information of WiPy as string"},
    {"versionMajor", versionMajor, METH_NOARGS, "return the major version information of WiPy"},
    {"versionMinor", versionMinor, METH_NOARGS, "return the minor version of WiPy"},
    {"versionBuild", versionBuild, METH_NOARGS, "return the build number of WiPy"},
	{"createRobot", createRobot, METH_NOARGS, "Create new robot if no robot exists. If a robot already exists (default), it is deleted and replace with a new instance."},
//	{"deleteRobot", deleteRobot, METH_NOARGS, "Destroy the current robot object."},	// we remove this function from the API as the user should not be able to delete the robot without generating a new one
    {NULL, NULL, 0, NULL}  
};
