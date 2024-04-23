/*
* WireX  -  WireCenter
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

/*! \file wcDllApi.h
 *
 *	\author   Andreas Pott
 *
 *  \dependency
 *		WireCenter		provide the application layer objects to be interfaced with
 *
 *  Declarations of interface functions required to setup the communication between 
 *  WireCenter and a dll that implements the plugin functions.
 */

#pragma once

#include<string>

#ifdef WCPLUGIN_WITHOUT_PYTHON
	// if we do not want to use python in the implementation of the plugin; we have to mimic
	// the pointer
	#define PyObject void
	typedef PyObject* (*PyCFunction )(PyObject*, PyObject*);
#else

	// we have some problems with the python debug library; so this should be a workaround
	#ifdef _DEBUG
	#undef _DEBUG
	#define NDEBUG
	#include <python/python.h>
	#undef NDEBUG
	#define _DEBUG
	#else
	#include <python/python.h>
	#endif // _DEBUG

#endif // WCPLUGIN_WITHOUT_PYTHON

/*! This structure is passed by the host poccess to the plug-in dll to provide 
 *  it with data from the application object. Thus, the IwcApp data strcuture 
 *  is normally filled by wireCenter to provide details of the running instance
 *  for the plugin, e.g. to access the current RobotDocument.
 *  We use void* for all data objects here. If the plugin actually makes use of 
 *  the data, typecasting is required.
 */
struct IwcApp
{
	//! object/pointer to access the python scripting tables of wireCenter
	void* pPyInterface;

	//! pointer to the current CRobotData object to intercae with the wire robot data model
	void* pRobotDoc;

	//! interface to 3-D-engine (IPAGL)
	void* pScene;

	//! interface to the applications GUI; espcially to the menubar and the message map; 
	//! the data type is not defined yet
	void* pGUI;

	//! data to receive the version of WireCenter to check for compatible memory layout
	std::string wcVersion;
};

/*! \struct IwcPlugin
  * contains data of the plugin to be interpreted by the running instance of WireCenter.
  * We use a void* here instead of PyMethodDef* and wcMethodTable* although we 
  * expect the table to be of that types. This is because we do not want 
  * to force the plugin to be depending on python or other techniques. Thus, we have to 
  * accept the addional overhead for typecasting.
  */
struct IwcPlugin
{
	//! the name of the plugin
	std::string name;
	//! a short desription of the plugin
	std::string desc;
	//! a version string
	std::string version;
	//! the symbolic name of the python module to be published for the python interpreter
	std::string pyModuleName;
	//! a pointer to the python method table with function adresses and meta information; see wcCommandTable*
	void *methodTable;
	//! the number of entries in the method map
	int methodCount;
	// more data might come
};


//! prototype of functions called through the invoke procedure in the plug-in's interface
//! prototype for a command to be called by the Invoke function of the API
typedef int (*PWCCOMMAND)(int, int);

//! prototype for the init function to start the 
typedef bool (WINAPI* PWCINITPLUGIN)(void*, void*);

//! prototype for the invoke function, that starts a WCCOMMAND from inside the dll, i.e. without exporting the function
typedef int (WINAPI* PWCINVOKE)(int, int, int);

//! prototype for an exported python function. all void* have to be cast to PyObject* when implementing the function
typedef PyObject* (WINAPI* PWCPYINVOKE)(char*, PyObject*, PyObject*);

//! just rename 
typedef PyCFunction PWCPYTHON ;

typedef enum { wcCommand=1, wcDllExported=2, wcPython=4 } wcAttributes;

//! data structure to be used as message map in the plugin
struct wcMethodTable
{
	char* name;				//!< symbolic name of a function provided by the plugin. the name will be used to symbolically identify the function. make sure that it coinsides with the name exported in the .def file
	char* desc;				//!< an optional desciption of the function
	char* help;				//!< further information about the function to be used by python
	PWCCOMMAND function;	//!< pointer to the function in the address space of the dll (do not call from WireCenter!)
	PWCPYTHON pyFunction;	//!< pointer to the python wrapper to be used by the python method table
	int attributes;			//!< a bit array with attributes.
};

//! connect WireCenter with the plugin DLL
//! these functions can be directly called from the host process
bool WINAPI wcInitPlugin(IwcApp*, IwcPlugin*);

int WINAPI wcInvoke(int id, int lparam, int rparam);

PyObject* WINAPI pyInvoke(char* function, PyObject* self, PyObject* arg);
