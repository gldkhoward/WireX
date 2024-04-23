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
 *  \file   : pyHelper.cpp
 *  \Author   Philipp Miermeister
 *  \Date     25.02.2014
 *	\brief Helper functions for python interface
 *********************************
 */

#include "motionPlanningPyBindings.h"

// some helper methods to simplify the handling of python objects

//! create a list of length "size" from array "values"
PyObject* pyToList(double* values, unsigned int size)
{
	PyObject* myList = PyList_New(0);
	if (!values)
		return myList;
	for (unsigned int i=0; i<size; i++)
		PyList_Append(myList,Py_BuildValue("d",values[i]));
	return myList;
}

//! create a list from the vector values
PyObject* pyToList(Vector3d& values)
{
	PyObject* myList = PyList_New(0);
	PyList_Append(myList,Py_BuildValue("d",values.x()));
	PyList_Append(myList,Py_BuildValue("d",values.y()));
	PyList_Append(myList,Py_BuildValue("d",values.z()));
	return myList;
}

//! create a tuple from the vector values
PyObject* pyToTuple(Vector3d& values)
{
	return Py_BuildValue("(ddd)",values.x(),values.y(),values.z());
}

//! create a list from the vector values1,values2
PyObject* pyToList(Vector3d& values1, Vector3d& values2)
{
	PyObject* myList = PyList_New(0);
	PyList_Append(myList,Py_BuildValue("d",values1.x()));
	PyList_Append(myList,Py_BuildValue("d",values1.y()));
	PyList_Append(myList,Py_BuildValue("d",values1.z()));
	PyList_Append(myList,Py_BuildValue("d",values2.x()));
	PyList_Append(myList,Py_BuildValue("d",values2.y()));
	PyList_Append(myList,Py_BuildValue("d",values2.z()));
	return myList;
}

//! create a list from the vector values
PyObject* pyToList(Matrix3d& values)
{
	MatrixXd R=values;
	return pyToList(R);
}

//! create a list from the matrix value
PyObject* pyToList(MatrixXd& value)
{
	PyObject* myList = PyList_New(0);
	if (value.cols()==1)
	{
		for (int i=0; i<value.rows(); i++)
			PyList_Append(myList,Py_BuildValue("d",value(i)));
	}
	else
	{
		for (int i=0; i<value.cols(); i++)
		{
			MatrixXd fi = value.col(i);
			PyList_Append(myList,pyToList(fi));
		}
	}
	return myList;
}

//! create a list from std::vector
PyObject* pyToList(vector<double> d_vector)
{
	PyObject* myList = PyList_New(0);
	if(!d_vector.empty())
	{
		unsigned int i=0;
		while(i<d_vector.size())
		{
			PyList_Append(myList,Py_BuildValue("d", d_vector.at(i)));
			i++;
		}
	}
	return myList;
}

//! fill an std::vector from python list
bool pyToVector(PyObject* obj, vector<double> *d_vector)
{
	if(!PyList_Check(obj))
		return false;
	int i=0;

	d_vector->clear();
	d_vector->resize(PyList_Size(obj));
	PyObject *iteration = PyObject_GetIter(obj);

	if (!iteration)
		return false;

	for ( PyObject* next=PyIter_Next(iteration); next; i++, next=PyIter_Next(iteration))
	{
		if (!PyFloat_Check(next))
		{
			if(PyInt_Check(next))
			{
				long l = PyInt_AsLong(next);
				d_vector->at(i)= (double)l;
			}
			else
				return false;
		}
		else
		{
			double item = PyFloat_AsDouble(next);
			d_vector->at(i)=item;
		}
	}
	return true;
}

//! return the python object that is defined as "true" value
PyObject* pyTrue() {
    Py_RETURN_TRUE;

    // return Py_BuildValue("i", 1);
}

//! return the python object that is defined as "true" value
PyObject* pyFalse() {
    Py_RETURN_FALSE;

    // return Py_BuildValue("i", 0);
}

//! return the python object that is defined as "none" value
PyObject* pyNone() {
    Py_RETURN_NONE;

    //return Py_BuildValue("i", 0);
}