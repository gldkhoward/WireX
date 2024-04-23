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

/*! \file PythonInterface.h
 *
 *	\author   Andreas Pott
 *
 *  \dependency
 *		Python      for scripting engine (strong dependency)
 *
 *  \brief 
 *		This class enables the python scripting inside an application.
 *		Through objects of the type CPythonInterface one can call 
 *		methods of the applications object. 
 */

#pragma once

#include <string>
#include <map>
#include <list>

// we have some problems with the python debug library; so this should be a workaround
#ifdef _DEBUG
#undef _DEBUG
#define NDEBUG
#include <python/python.h>
#include <Python/frameobject.h>
#undef NDEBUG
#define _DEBUG
#else
#include <python/python.h>
#include <Python/frameobject.h>
#endif // _DEBUG


/*! \brief interface class to enable python scripting 
 *  and to publish the exported functions in python moduls.
 *  \remark CPythonInterface is implemented as singleton, i.e. guarantee that 
 *  we have only one instance of this class. 
 */
class CPythonInterface
{
	//! filename of the build-in system scripts, i.e. scripts that can be linked to events and menu commands
	std::string systemscript;	
	//! prefix used as convention to recognize python functions that are events
	std::string eventPrefix;
	//! prefix used as convention to recognize python functions that are actions
	std::string actionPrefix;
	//! a list with all events provided by the system script
	std::list<string> events;
	//! a list with all actions provided by the system script
	std::list<string> actions;
	//! a call back function to handle the events generated from the python interpreter
	static int TraceCallback(PyObject* obj, PyFrameObject* frame, int what, PyObject* arg);
	//! a list with the python method tables mapping the symbolic module names to the pointers of the method tables
	std::map<std::string, PyMethodDef*> ModuleTable;
	//! a mapping of function names to docstrings
	std::map<std::string, std::string> docstrings;

	PyThreadState* mainThreadState;
	char** argv;

public:
	bool bEnableProfile;	//!< set to true for debug purpose in order to receive interpreter information on calling methods
	bool bEnableTrace;		//!< set to true for debug purpose, Python will call frequenctly a report function while interpreting scripts

private:
	//! If you get an error for one of the following lines of code ("cannot access
	//! private member...") you have to use CPythonInterface::getInstance() 
	//! instead of trying to create your own instance! CPythonInterface is a singelton
	//! and thus instances cannot be generated.
	CPythonInterface(void);
	~CPythonInterface(void);
	//! copy is forbitten for a singleton
	CPythonInterface(const CPythonInterface&) {}							
	//! singletons cannot be asigned
	CPythonInterface& operator=(const CPythonInterface&) { return *this; }	

public:
	//! return the one and only instance of the class
	static CPythonInterface& getInstance();
private: // internal function of the interface
	//! initialize the python infrastructure
	bool initPythonInterface();
	//!
//	void getPointer(PyMethodDef*& pIapp, PyMethodDef*& pIrobot);
	//! create a Python scripts which imports all registered python modules. 
	bool createPythonIncludeScript();
	//! helper function to prepare the script prior to opening the script
	bool openScript(const std::string& filename, PyObject*& pModule) const;

public:	// the public interface or API of this class to be used with getInstance()

	//! add a python method table to the current instance
	bool addMethodTable(std::string name, PyMethodDef* table);
	//! receive a constant reference for the method table
	const std::map<std::string, PyMethodDef*>&  getModuleTable() const { return ModuleTable; }

	//! start a method from a python script
	bool run(const std::string& filename, const std::string& method);
	//! start a simple command string
	bool runCommand(const std::string& command);
	//! marshal serialized python object
	bool marshal(const std::string& strObject_in, std::string& strObject_out);
	//! start a method in the system script
	bool runSystemScript(const std::string& method);
	//! reset python interpreter
	bool reset();
	//! init system script
	bool initSystemScript();
	//! return a list if all actions in the system script
	bool getActions(list<string>& actions)
		{ actions=this->actions;  return true; }
	bool getActionMap(map<string,string>& actionMap)
		{ actionMap = docstrings; return true; }

public: // services provided by CPythonInterface
	//! parse a python script and return a list of all function names (as strings) inside the script
	bool getCallableFunctions(const std::string& filename, std::list<std::string>& functionNames, std::list<std::string>& docstrings) const;
};

