/*
* WireX  -  WireLibPyBindings
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

/*! \file WireLibPythonBindings.h
 *
 *	\author  Philipp Miermeister
 *
 *  \dependency
 *		Eigen3		for IR� algebra
 *		tinyXML		for parameter IO
 *
 *	\brief Python bindings for the WireLib module
 */

#pragma once
#include <WireLib/RobotDocument.h>

#ifndef WIN32
// change from python2.7 to python3.6
#include <python3.10/Python.h>

#else

// we have some problems with the python debug library; so this should be a workaround
#ifdef _DEBUG
#undef _DEBUG
#define NDEBUG
#include <Python/Python.h>
#undef NDEBUG
#define _DEBUG
#else
#include <Python/Python.h>
#endif // _DEBUG

#endif // WIN32

// This method must be implemented by the respective import module to create the robot document object
PCRL::CRobotDocument* getRobotDocument();

//PCRL::CRobotDocument* pRobotDoc;
//extern PCRL::CRobotDocument* pRobotDoc;
PCRL::CWorkspaceDifferentialHull* pyHelperHull();
PCRL::CWorkspaceGrid* pyHelperGrid();
PCRL::CForceDistribution* pyHelperFD();
PCRL::CWorkspaceCrosssection* pyHelperCS();
PCRL::CElastoKinematics* pyHelperKinematics();
PCRL::CStiffness* pyHelperStiffness();
PCRL::CInterference* pyHelperInterference();
PCRL::CRobotData* pyHelperRobot();
PCRL::CApplicationRequirement* pyHelperApp();

// Some convenience functions
PyObject* pyToList(double* values, unsigned int size);
PyObject* pyToTuple(Vector3d& values);
PyObject* pyToList(Vector3d& values);
PyObject* pyToList(Vector3d& values1, Vector3d& values2);
PyObject* pyToList(Matrix3d& values);
PyObject* pyToList(MatrixXd& value);
PyObject* pyToList(vector<double> d_vector);
bool pyToVector(PyObject* obj, vector<double> *d_vector);

//PyObject* pyTrue();
//PyObject* pyFalse();

// Method tables
extern PyMethodDef EmbMethodsIrobot[];
extern PyMethodDef EmbMethodsIkin[];
extern PyMethodDef EmbMethodsIws[];
extern PyMethodDef EmbMethodsControl[];

// convinient macros to simplify the definition of trivial wrappers without logical options
// beside making the implementation of such wrapper simple the macros generate a 
// standarized behavior amongst different functions

// helper for calling functions without arguments and return values. this can be used to dispatch simple commands
// add the brackets to the function to be called as arguments from the global scope can be passed to the function
#define PYHELPER_VOID_VOID(FncName, callableObject) \
static PyObject* FncName(PyObject *self, PyObject *args) { callableObject; Py_RETURN_NONE;  }

#define PYHELPER_BOOL_VOID(FncName, callableObject) \
static PyObject* FncName(PyObject *self, PyObject *args) { if ( callableObject ) Py_RETURN_TRUE ; Py_RETURN_FALSE ; }

//! helper for calling a function with a simple string argument without return value
#define PYHELPER_VOID_STRING(FncName, callableObject) \
	static PyObject* FncName(PyObject *self, PyObject *args) { \
	char* str; \
	if ( ! PyArg_ParseTuple(args, "s",&str) ) return NULL; \
	callableObject(str); \
	Py_RETURN_NONE; } 

//! helper for calling a function with a simple string argument with a boolean return value
#define PYHELPER_BOOL_STRING(FncName, callableObject) \
	static PyObject* FncName(PyObject *self, PyObject *args) { \
	char* str; \
	if ( ! PyArg_ParseTuple(args, "s",&str) ) return NULL; \
	if (callableObject(str)) Py_RETURN_TRUE; \
	Py_RETURN_FALSE; } 
