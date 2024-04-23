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

/*! \file motionPlanningPyBindings.h
 *
 *	\author  Philipp Miermeister, Andreas Pott
 *
 *  \Date     20.05.2015
 *
 *  \dependency
 *		Eigen3			for IR³ algebra
 *		tinyXML			for parameter IO
 *      motionPlanning	the library  for which this wrapper is provided
 *
 *	\brief Python bindings for the motionPlanning module
 */

#pragma once
#include <motionPlanning/NcDoc.h>

#ifndef WIN32
#include <python2.7/Python.h>

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
PCRL::CNcDoc* getDoc();

// Some convenience functions
PyObject* pyToList(double* values, unsigned int size);
PyObject* pyToTuple(Vector3d& values);
PyObject* pyToList(Vector3d& values);
PyObject* pyToList(Vector3d& values1, Vector3d& values2);
PyObject* pyToList(Matrix3d& values);
PyObject* pyToList(MatrixXd& value);
PyObject* pyToList(vector<double> d_vector);
bool pyToVector(PyObject* obj, vector<double> *d_vector);

PyObject* pyTrue();
PyObject* pyFalse();

// Method tables
extern PyMethodDef EmbMethodsControl[];

