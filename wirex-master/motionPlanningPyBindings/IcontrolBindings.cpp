/*
* WireX  -  motionPlanningPyBindings
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

/*!********************************
 *  \file   : IcontrolBindings.cpp
 *  \Author   Philipp Miermeister
 *  \Date     25.02.2014
 *	\brief	  This file contains all python bindings related to robot control
 *********************************
 */ 

#include "motionPlanningPyBindings.h"


/* */
static PyObject* WireRobot_ncSetAmax(PyObject *self, PyObject *args)
{

	if (getDoc()==NULL)
	{
		cout << "Error. Please initialize robot first."<< endl;
		Py_RETURN_NONE;
	}

	double amax;

	if ( ! PyArg_ParseTuple(args, "d",&amax) )
        return NULL;

	if ( getDoc()->Interpolator.setAmax(amax) )
        Py_RETURN_TRUE;

	Py_RETURN_FALSE;
}

/* */
static PyObject* WireRobot_ncSetJmax(PyObject *self, PyObject *args)
{

	if (getDoc()==NULL)
	{
		cout << "Error. Please initialize robot first."<< endl;
		Py_RETURN_NONE;
	}

	double jmax;

	if ( ! PyArg_ParseTuple(args, "d",&jmax) )
        return NULL;

	if ( getDoc()->Interpolator.setJmax(jmax) )
        Py_RETURN_TRUE;

	Py_RETURN_FALSE;
}

/* */
static PyObject* WireRobot_ncSetCycleTime(PyObject *self, PyObject *args)
{
	if (getDoc()==NULL)
	{
		cout << "Error. Please initialize robot first."<< endl;
		Py_RETURN_NONE;
	}

	double cycleTime;

	if ( ! PyArg_ParseTuple(args, "d",&cycleTime) )
        return NULL;

	if ( getDoc()->Interpolator.setCycleTime(cycleTime) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* */
static PyObject* WireRobot_ncPrintTrajectoryParameters(PyObject *self, PyObject *args)
{
	if (getDoc()==NULL)
	{
		cout << "Error. Please initialize robot first."<< endl;
		Py_RETURN_NONE;
	}
		
	getDoc()->Interpolator.printTrajectoryParameters();
	
    Py_RETURN_NONE;
}


/* */
static PyObject* WireRobot_ncSetInterpolationMethod(PyObject *self, PyObject *args)
{
	if (getDoc()==NULL)
	{
		cout << "Error. Please initialize robot first."<< endl;
		Py_RETURN_NONE;
	}

	int iInterpolationMethod;
	
	if ( ! PyArg_ParseTuple(args, "i",&iInterpolationMethod) )
        return NULL;

	if ( getDoc()->Interpolator.setInterpolationMethod(iInterpolationMethod) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* */
static PyObject* WireRobot_ncSetBezier(PyObject *self, PyObject *args)
{
	
	if (getDoc()==NULL)
	{
		cout << "Error. Please initialize robot first."<< endl;
		Py_RETURN_NONE;
	}

	int iBezier;
	
	if ( ! PyArg_ParseTuple(args, "i",&iBezier) )
        return NULL;

	if ( getDoc()->Interpolator.setBezier(iBezier) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* */
static PyObject* WireRobot_ncSetOverride(PyObject *self, PyObject *args)
{

	if (getDoc()==NULL)
	{
		cout << "Error. Please initialize robot first."<< endl;
		Py_RETURN_NONE;
	}

	double dOverride;

	if ( ! PyArg_ParseTuple(args, "d",&dOverride) )
        return NULL;

	if ( getDoc()->Interpolator.setOverride(dOverride) )
	    Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* */
static PyObject* WireRobot_ncSetStartPosition(PyObject *self, PyObject *args)
{

	if (getDoc()==NULL)
	{
		cout << "Error. Please initialize robot first."<< endl;
		Py_RETURN_NONE;
	}

	Vector3d pos(0,0,0);


	if ( ! PyArg_ParseTuple(args, "|ddd",&pos.x(),&pos.y(),&pos.z()) )
        return NULL;

	pos = pos*1000; // input by user in [m], NC-Interpolator works internally in [mm]
	
    if ( getDoc()->Interpolator.setStartPosition(pos) )
	    Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* */
static PyObject* WireRobot_ncSetStartOrientation(PyObject *self, PyObject *args)
{
	
	if (getDoc()==NULL)
	{
		cout << "Error. Please initialize robot first."<< endl;
		Py_RETURN_NONE;
	}

	Vector3d pos(0,0,0);

	if ( ! PyArg_ParseTuple(args, "|ddd",&pos.x(),&pos.y(),&pos.z()) )
        return NULL;

	if ( getDoc()->Interpolator.setStartOrientation(pos) )
	    Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* */
static PyObject* WireRobot_ncLoadNCProgram(PyObject *self, PyObject *args)
{
	if (getDoc()==NULL)
	{
		cout << "Error. Please initialize robot first."<< endl;
		Py_RETURN_NONE;
	}

	char *filename;
	if ( ! PyArg_ParseTuple(args, "s",&filename) )
        return NULL;

	if ( getDoc()->loadNcProgram(filename) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* */
static PyObject* WireRobot_ncParseNCProgram(PyObject *self, PyObject *args)
{
	if (getDoc()==NULL)
	{
		cout << "Error. Please initialize robot first."<< endl;
		Py_RETURN_NONE;
	}

	char *filename;
	if ( ! PyArg_ParseTuple(args, "s",&filename) )
        return NULL;

	if ( getDoc()->parseNcProgram(filename) )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* */
static PyObject* WireRobot_ncInterpolateNCProgram(PyObject *self, PyObject *args)
{
	if (getDoc()==NULL)
	{
		cout << "Error. Please initialize robot first."<< endl;
		Py_RETURN_NONE;
	}

	if ( getDoc()->interpolateNcProgram() )
        Py_RETURN_TRUE;

    Py_RETURN_FALSE;
}

/* */
static PyObject* WireRobot_ncExportPoselist(PyObject *self, PyObject *args)
{
	if (getDoc()==NULL)
	{
		cout << "Error. Please initialize robot first."<< endl;
		Py_RETURN_NONE;
	}

	char *sFilename;
	if ( ! PyArg_ParseTuple(args, "s",&sFilename) )
        return NULL;

	getDoc()->PoseList.SavePoseListToFile(IOCSV,sFilename);

	Py_RETURN_NONE;
}

/* */
static PyObject* WireRobot_ncImportPoselist(PyObject *self, PyObject *args)
{
	if (getDoc()==NULL)
	{
		cout << "Error. Please initialize robot first."<< endl;
		Py_RETURN_NONE;
	}

	char *sFilename;
	int iFormat=1;
	if ( ! PyArg_ParseTuple(args, "s|i",&sFilename, &iFormat) )
        return NULL;

	getDoc()->PoseList.LoadPoseListFromFile(sFilename, iFormat);

	Py_RETURN_NONE;
}

// Create method table and module help
//------------------------------------------------------------------------

static PyObject* WireRobot_HelpControl(PyObject *self, PyObject *args);

PyMethodDef EmbMethodsControl[] = {
	{"help", WireRobot_HelpControl, METH_NOARGS,  "Print the names of all methods in Icontrol [void]"},

	{"ncSetAmax", WireRobot_ncSetAmax, METH_VARARGS,  "set the max acceleration in m/s^2"},
    {"ncSetBezier", WireRobot_ncSetBezier, METH_VARARGS,  "enable or disable Bezier 1=activated"},
    {"ncSetCycleTime", WireRobot_ncSetCycleTime, METH_VARARGS,  "set the cycle time of the interpolator in ms"},
	{"ncSetInterpolationMethod", WireRobot_ncSetInterpolationMethod, METH_VARARGS,  "set the interpolation method: 1= simple interpolator without dynamic ; 2= high sophisticated interpolator with acceleration ramps"},
    {"ncSetJmax", WireRobot_ncSetJmax, METH_VARARGS,  "set the max jerk in m/s^3"},
	{"ncSetOverride", WireRobot_ncSetOverride, METH_VARARGS,  "set the override in % to scale the max. velocity"},
    {"ncSetStartOrientation", WireRobot_ncSetStartOrientation, METH_VARARGS,  "set the initial orientation of the NC path [a,b,c]"},
	{"ncSetStartPosition", WireRobot_ncSetStartPosition, METH_VARARGS,  "set the initial position of the NC path [x,y,z]"},
    {"ncPrintTrajectoryParameters", WireRobot_ncPrintTrajectoryParameters, METH_NOARGS,  "get the trajectory parameters"},
    
    {"ncExportPoselist", WireRobot_ncExportPoselist, METH_VARARGS,  "export the poselist as csv [filename]"},
    {"ncImportPoselist", WireRobot_ncImportPoselist, METH_VARARGS,  "import a poselist from csv [filename, format]"},
    {"ncLoadProgram", WireRobot_ncLoadNCProgram, METH_VARARGS,  "parse and interpolate a NC-File"},
	{"ncParseProgram", WireRobot_ncParseNCProgram, METH_VARARGS,  "parse a NC-File"},
	{"ncInterpolateProgram", WireRobot_ncInterpolateNCProgram, METH_NOARGS,  "interpolate a NC-File"},

	{NULL, NULL, 0, NULL}
};


/* print the full method table of the Icontrol object */
static PyObject* WireRobot_HelpControl(PyObject *self, PyObject *args)
{
	for ( PyMethodDef* ptr = EmbMethodsControl; ptr->ml_name != 0; ptr++ )
        PySys_WriteStdout("Icontrol.%s: %s\n", ptr->ml_name, ptr->ml_doc);

	Py_RETURN_NONE;
}