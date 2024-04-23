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

// PythonInterface.cpp: Implementierungsdatei
//

#include "StdAfx.h"
#include "PythonInterface.h"
#include <fstream>
#include "wcPyBindings.h "
#include <Python/frameobject.h>

// python helper functions for rediction of pythons console output to printf calls
// one can easily exchange the printf calls with other appropriate handling of the
// data, e.g. writing them to a log, a file, or sending over a socket connection.

PyObject* log_CaptureStdout(PyObject* self, PyObject* pArgs)
{
	char* LogStr = NULL;
	if (!PyArg_ParseTuple(pArgs, "s", &LogStr)) return NULL;

	printf("%s", LogStr); //output python object string

	Py_INCREF(Py_None);
	return Py_None; //returning 0 will give error
}

// Notice we have STDERR too.
PyObject* log_CaptureStderr(PyObject* self, PyObject* pArgs)
{
	char* LogStr = NULL;
	if (!PyArg_ParseTuple(pArgs, "s", &LogStr)) return NULL;

	printf("%s", LogStr);

	Py_INCREF(Py_None);
	return Py_None;
}

static PyMethodDef logMethods[] = {
 {"CaptureStdout", log_CaptureStdout, METH_VARARGS, "Logs stdout"},
 {"CaptureStderr", log_CaptureStderr, METH_VARARGS, "Logs stderr"},
 {NULL, NULL, 0, NULL}
};

///////////////////////////////////////////////////////////
// Class CPythonInterface
///////////////////////////////////////////////////////////

//! obj containts the value pass to when this call-back was attached
//! what identifies the reseon for being called back
int CPythonInterface::TraceCallback(PyObject* obj, PyFrameObject* frame, int what, PyObject* arg)
{
	printf("Trace %i: ",clock());
	switch (what) {
	case PyTrace_CALL:		printf("CALL"); break;
	case PyTrace_C_CALL:	printf("C_CALL"); break;
	case PyTrace_C_RETURN:	printf("C_RETURN"); break;
	case PyTrace_LINE:		printf("LINE"); break;
	case PyTrace_RETURN:	printf("RETURN"); break;
	default: 
		printf("unknwon event in 'what' (what=%i) | ",what);
	}
	// we can get the current code line from inspecting the frame
	printf(" Line %i | ",PyFrame_GetLineNumber(frame));
	printf(" %s | ",PyString_AsString(frame->f_code->co_filename));
	printf(" %s | ",PyString_AsString(frame->f_code->co_name));

	// we convert the arg parameter in a string representation
	PyObject* rep = PyObject_Repr(arg);
	if (PyModule_Check(arg))
	{
		printf("Module Name: %s |",PyModule_GetName(arg));
	}
	if (PyFunction_Check(arg))
	{
		//
	}
	if (PyCallable_Check(arg))
		printf("Function: %s",PyString_AsString(rep));
	else
		printf("args: %s",PyString_AsString(rep));
	printf("\n");
	return 0;
}

//! the Python interface class initialize the python library and the exported 
//! module function table.
CPythonInterface::CPythonInterface()
{
//! \todo FIXME: The update to python 2.7.3.1 caused a problem with the call to
//! Py_Initialize()
//! We must either set the home variable during runtime or in the project settings
//! of WireCenter.sln under Configuration Properties->Debugging->Environment
//! add e.g.  PYTHONHOME=C:\Portable\Portable Python 2.7.3.1\App\;
//! or whereever your python standard libs are stored. 

	// print startup infomation
	cout << Py_GetVersion() << endl;

	argv = (char**)malloc(sizeof(char*) * 1);
    argv[0]     = (char*)malloc(sizeof(char)  * 256);
	GetModuleFileName(0, argv[0], 256);
	
	Py_SetProgramName(*argv);
	if (Py_GetPythonHome())
		cout << "Python Path: " << Py_GetPythonHome() << endl;

	//! \todo FIXME: These parameters should be set depending on some configuration parameters
	Py_OptimizeFlag = 0;// 0: do not optimize but maintain docstrings. 2:set python to optimized mode simiar to compiler option -o2
	Py_NoSiteFlag = 1;	 // ignore __site__.py file
    Py_NoUserSiteDirectory = 1;
	Py_Initialize();
	cout << "Python interpreter loaded\n";
	//! end of problem section; remove this line after the bug was fixed!

	//Start New thread
	mainThreadState = Py_NewInterpreter();
	PySys_SetArgvEx(1,argv, 0);


	ASSERT(Py_IsInitialized());
	systemscript="wirecenter.py"; //move this
	eventPrefix="On";
	actionPrefix="action";
	bEnableProfile = false;
	bEnableTrace = false;
	
	// experimentally add a trace function
	if (bEnableTrace)
		PyEval_SetTrace(TraceCallback, NULL);
	if (bEnableProfile)
		PyEval_SetProfile(TraceCallback, NULL);

	initPythonInterface();
	createPythonIncludeScript();
	//free(argv);
}

//! setup the method table, register the python callbacks 
bool CPythonInterface::initPythonInterface()
{
	char buffer[MAX_PATH];
	std::string::size_type pos; 
	std::string PythonRunPath; 
	
	GetModuleFileName(NULL, buffer, MAX_PATH);

	pos = std::string( buffer ).find_last_of( "\\/" );
	PythonRunPath = string( buffer ).substr( 0, pos);
	
	size_t start_pos=0;
	while((start_pos = PythonRunPath.find("\\", start_pos)) != std::string::npos) {
		PythonRunPath.replace(start_pos,1, "/");
        start_pos += 1; //
    }

	// additionally we initialize some internal module tables that are not published through the ModuleTable data structure
	// redirect the output streams such that python output to the console can be used by the host process
	Py_InitModule("log",logMethods);
	 
	if (0 != PyRun_SimpleString(   //configure python to use ["log"]=logMethods functions instead of sys.stdout.
		"import log\n"
		"import sys\n"
		"class StdoutCatcher:\n"
		"\tdef write(self, str):\n"
		"\t\tlog.CaptureStdout(str)\n"
		"class StderrCatcher:\n"
		"\tdef write(self, str):\n"
		"\t\tlog.CaptureStderr(str)\n"
		"\tdef flush(self):\n"
		"\t\tpass\n"
		"sys.stdout = StdoutCatcher()\n"
		"sys.stderr = StderrCatcher()\n"
	)) //configure python to use ["log"]=logMethods functions instead of sys.stdout.
		printf("Error while redirecting the python output stream to the console window.\n"
			   "WARNING: Python will not be able to print and tell you about errors!\n");

	if (0 == PyRun_SimpleString("import os\n"))
	{
		// wrap the extracted path with a change dir statement
		PythonRunPath.insert(0,	"os.chdir(\"");
		PythonRunPath.append("\")");
		PyRun_SimpleString(PythonRunPath.c_str());
	}

	return true;
}

CPythonInterface::~CPythonInterface()
{
	// paranoia: clear possible unhandled exceptions in python before finalizing
	PyErr_Clear();
	//Py_EndInterpreter(mainThreadState);
	//End Thread, Py_Finalize destroys all open interpreters: http://docs.python.org/2/c-api/init.html#Py_Finalize
	Py_Finalize();
	free(argv);
}

/*! this is a helper function to prepare and open a script. If the function successfully
 *  extracts a module, the caller is responsible to dispose the python object (e.g. to
 *  call DECREF(...) or similar )
 *  This function copies the desired script to the local directory and opens it from this
 *  location. It seems a possibility to cope with unexpected problems.
 *  \param filename [in] the filename as a string of the file to be opened. 
 *  \param pModule [out] containts the PyObject representing the loaded module, i.e. the script
 *  \return true, if successful. if false is return the value of pModule is undefined
 */
bool CPythonInterface::openScript(const std::string& filename, PyObject*& pModule) const
{
	if (!Py_IsInitialized())
		return false;
	// copy the python script into the path of the exe
	char path[MAX_PATH + 1];
	// get the Win32 API to get the path of the host process

	GetModuleFileName( NULL, path, MAX_PATH + 1 );
	std::string sPath(path);
	sPath=sPath.substr(0,sPath.rfind("\\"));
    sPath+="\\tmp.py";
	//Where is this looking?
	if (!CopyFile(filename.c_str(), sPath.c_str(), false))
		printf("Error in Python Script: The python script file could not be copied to temp file.");

	// open the script file from the harddisk
	PyObject* pName = PyString_FromString("tmp");
	if (!pName)
	{
		PyErr_Print();
		return false; 
	}
	// now we can open the module
	pModule = PyImport_Import(pName);
	Py_DECREF(pName);

	if (!pModule)
		return false;

	return true;
}

/*! extract all callable functions name from the python script filename
 *  \remark the implementation is quite redundant with "run" and might need some
 *          shared source code in an internal function to open the python file
 */
bool CPythonInterface::getCallableFunctions(const std::string& filename, std::list<std::string>& functionNames, std::list<std::string>& docstrings) const
{
	PyObject* pModule=0;
	if (!openScript(filename,pModule))
		return false;

	// iterate through the global dict of the module and search for the 
	// functions defined in the module
	PyObject* pDict = PyModule_GetDict(pModule);
	if (pDict!=NULL)
	{
		PyObject* pList = PyDict_Items(pDict);
		for (int i=0; i<PyList_Size(pList); i++)
		{
			// get the next item in the list extracted from the dict
			PyObject* pItem = PyList_GetItem(pList,i);
			// we are looking for tuple entries with two objects inside, where the second object to of type "callable"
			if (PyTuple_CheckExact(pItem))
			{
				if (PyTuple_Size(pItem)==2)
				{
					PyObject *first = PyTuple_GetItem(pItem,0);
					PyObject *second = PyTuple_GetItem(pItem,1);

					// try to get the docu string to complete the information in the console/action tab.
					// this is possible if the optimization level for python is 0 (rather than 1 or 2)
					//
					// the following code extracts the docString. However, we lack an implementation
					// that forwards this docstring to the respective control to show it 
					PyObject *doc = PyObject_GetAttrString(second,"__doc__");
					// store the name of the callable function in the functionName list
					if (PyCallable_Check(second))
					{
						functionNames.push_back(PyString_AsString(PyObject_Str(first)));
						if (doc && PyString_AsString(doc))
							docstrings.push_back(PyString_AsString(doc));
						else
							docstrings.push_back("no documentation");	// if no docstring was found, we add an empty string
					}
				}
			}
		}
	}

	Py_DECREF(pModule);
	return true;
}

/*! init system script
 *  during initializtation the system script is inspected and all callable functions
 *  are extracted. Then we search these functions for so-called events and actions.
 *  Events are connected to some conditions (usually functions called by the application
 *  framework). Such event handler in the C++-program may start with a line of code
 *  calling a python script that is execute whenever this application function is 
 *  started. Python functions used as events extended or customize the behaviour of
 *  functions that are already integrated in the GUI.
 *  Contrary, actions are provided as callable extensions to the application offering
 *  additional services. Adding such actions is adhoc. While the implementation of the
 *  GUI needs to pre-define all events, one can add infinitely may additional actions.
 *  Action are typically identified at runtime.
 *
 *  \return true, if the system script was successfully parsed and analyzed. 
 */
bool CPythonInterface::initSystemScript()
{
	// clean up the cached names
	actions.clear();
	events.clear();

	list<string> fncNames,docs;
	//! inspect the script to find all callable function
	if (!getCallableFunctions(systemscript,fncNames,docs))
		return false;

	ASSERT(fncNames.size()!=docstrings.size());

	//! sort these function into two piles: put actions and events into the respective 
	for (list<string>::iterator itor = fncNames.begin(), itor2=docs.begin(); itor!=fncNames.end(); itor++, itor2++)
	{
		if ((*itor).compare(0,eventPrefix.length(),eventPrefix)==0)
			events.push_back(*itor);
		if ((*itor).compare(0,actionPrefix.length(),actionPrefix)==0)
		{
			actions.push_back(*itor);
			docstrings[*itor]=*itor2;
		}
	}
	return true;
}

/*! Start the function "method" of a python script with "filename" from 
 *  the c++ program. The python script can use the above defined functions 
 *  to interact with the c++ GUI.
 */
bool CPythonInterface::run(const std::string& filename, const std::string& method)
{
/*	// We should use the openScript function here but it does not work for some reason
	PyObject* pModule=0;
	if (!openScript(filename, pModule))
		return false;*/

	if (!Py_IsInitialized())
		return false;
	// copy the python script into the path of the exe
	char path[MAX_PATH + 1];
	// get the Win32 API to get the path of the host process
    GetModuleFileName( NULL, path, MAX_PATH + 1 );
	std::string sPath(path);
	sPath=sPath.substr(0,sPath.rfind("\\"));
    sPath+="\\tmp.py";
	if (!CopyFile(filename.c_str(), sPath.c_str(), false))
		printf("Error in Pyhton Script: The Python script file could not be copied to temp.");

	// open the script file from the harddisk
	PyObject* pName = PyString_FromString("tmp");
	if (!pName)
	{
		PyErr_Print();
		return false; 
	}
	PyObject* pModule = PyImport_Import(pName);
	Py_DECREF(pName);

	// EXPERIMENTAL BEGIN ***
#ifdef USE_PYTHON_REFLECTION_ON_SYSTEM_SCRIPT
	PyObject* pDict = PyModule_GetDict(pModule);
	if (pDict!=NULL)
	{
		fprintf(stdout,"Inspection of the module begins\n");
		PyObject* pList = PyDict_Items(pDict);
		for (int i=0; i<PyList_Size(pList); i++)
		{
			PyObject* pItem = PyList_GetItem(pList,i);
			// iterate through the items in the dict; however the callable functions itself 
			// might be only part of what is inside pItem
			if (PyCallable_Check(pItem))
				cout << "Callable: ";
			if (PyList_CheckExact(pItem))
				cout << "List ";
			if (PyTuple_CheckExact(pItem))
			{
				if (PyTuple_Size(pItem)==2)
				{
					PyObject *first = PyTuple_GetItem(pItem,0);
					PyObject *second = PyTuple_GetItem(pItem,1);
					if (PyCallable_Check(second))
					{
						cout
							<< "Function: " 
							<< PyString_AsString(PyObject_Str(first)) << "(...) ";
					}
				}
				// we can iterature through the tuple
				cout << "Tuple ((";
				cout << PyTuple_Size(pItem) << " :: ";
				for (int j=0; j<PyTuple_Size(pItem); j++)
				{
					PyObject* pTupleItem = PyTuple_GetItem(pItem,j);
					cout << PyString_AsString(PyObject_Str(pTupleItem)) << " || ";
				}
				cout << " )) ";
			}
			if (PyDictItems_Check(pItem))
				cout << "Dict ";
			PyObject* res = PyObject_Str(pItem);
			cout << PyString_AsString(res) << endl;
		}
	}

	PyObject* pLocals = PyEval_GetLocals();
	// EXPERIMENTAL END ***
#endif //USE_PYTHON_REFLECTION_ON_SYSTEM_SCRIPT

	if (pModule!=NULL)
	{
		// search for the function to be called
		PyObject* pFunc = PyObject_GetAttrString(pModule, method.c_str());

		// call if function can be called
		if (pFunc && PyCallable_Check(pFunc))
		{
			// call the function
			PyObject* pValue = PyObject_CallObject(pFunc, NULL);
			if (pValue!=NULL)
				Py_DECREF(pValue);
			else
			{
				PyErr_Print();
				// clean up
				Py_XDECREF(pFunc);
				Py_DECREF(pModule);		
				return false;
			}
		}
		else
		{
			PyErr_Print();
			// clean up
			Py_XDECREF(pFunc);
			Py_DECREF(pModule);		
			return false;
		}
		// clean up
		Py_XDECREF(pFunc);
		Py_DECREF(pModule);		
	}
	else
	{	
		PyErr_Print();
		PyErr_Clear();
		return false;
	}

	return true;
}

//! start a simple command string
//! \todo Here are some names (IWC) and commands "Iapp.invalidate()" that must
//! be exchanged for parameters in order to make the implementation indepdentent
//! from a specific software framework.
bool CPythonInterface::runCommand(const std::string& command)
{
	// auto include all created models before executing the string
	std::string str("from IWC import *\n");
	str+=command;
	str+="\nIapp.invalidate()\n";
	if (PyRun_SimpleString(str.c_str())==0)
		return true;
	printf("Error while involving the python interpreter.\n");
	PyErr_Print();
	return false;
}

//! marshal and evaluate serialized python objects
bool CPythonInterface::marshal(const std::string& strObject_in, std::string &strObject_out)
{
	std::string str("from IWC import *\n");
	str+="import marshal\n";
	str+="\nIapp.invalidate()\n";
	if (!PyRun_SimpleString(str.c_str())==0)
	{
		printf("Error while involving the python interpreter.\n");
		PyErr_Print();
		return false;
	}
	// get reference to main module
	PyObject * mainModule = PyImport_AddModule("__main__");
	// create python string object
	PyObject* pStr = Py_BuildValue("s#", strObject_in.c_str(),strObject_in.length()); //s# has to be used to deal with strings that contain NULL values
	// marshall and evaluate function
	PyObject_SetAttrString(mainModule, "objStr", pStr);
	//str = "print objStr.encode('hex')\n";
	str = "f_tuple = marshal.loads(objStr)\n"; //deserialize object string
	str += "if (f_tuple[0] == 'exec'): exec(str(f_tuple[1]))\n";
	str += "else: fRes = eval(f_tuple[0]+str(f_tuple[1:]).rstrip(',)')+')')\n"; // evaluate python function (strip traling commas to avoid problems with one-tuples)
	str += "objStr_out=marshal.dumps(fRes)\n";		//serialize object string
	if (!PyRun_SimpleString(str.c_str())==0)
	{
		printf("Error while involving the python interpreter.\n");
		PyErr_Print();
		return false;
	}
	PyObject* pStr_out = PyObject_GetAttrString(mainModule, "objStr_out");
	if (pStr_out == NULL){ printf("Error: pStr_out is NULL"); return false;}
	
	char* pChar;
	int pyStringSize = 0;
	PyString_AsStringAndSize(pStr_out, &pChar, &pyStringSize);
	strObject_out = std::string(pChar,pyStringSize);
	return true;
}

/*! create a Python scripts which imports all registered python modules. 
 *  to use the modules in any python script write "from IWC import*"
 *  the function is called by the constructor of the PythonInterface
 *  \todo Again, here is a hard coded name for the top level module that
 *  should be exchanged with a parameter. 
 */
bool CPythonInterface::createPythonIncludeScript()
{
	// get the path of the exe
	char path[MAX_PATH + 1];
	// use the Win32 API to get the path of the host process
    GetModuleFileName( NULL, path, MAX_PATH + 1 );
	std::string sPath(path);
	sPath=sPath.substr(0,sPath.rfind("\\"));
    sPath=sPath + "\\IWC.py";

	// create a little python script with the import commands for the registered modules
	std::ofstream file_out(sPath.c_str());
	std::map<std::string, PyMethodDef*>::const_iterator itor;
    for (itor=ModuleTable.begin(); itor!=ModuleTable.end(); itor++)
		file_out << "import " << itor->first.c_str() << "\n" ;

	return true;
}

//! start a method in the system script
bool CPythonInterface::runSystemScript(const std::string& method)
{
	if (systemscript.length()<=0)
		return false;
	if (run(systemscript,method))
		return true;
	printf("Error while involving the python system script %s in file %s\n",method.c_str(),systemscript.c_str());
	return false;
}

//! add a new function table to the python interface
bool CPythonInterface::addMethodTable(std::string name, PyMethodDef* table)
{
	// add the symbolic name and the mobule table to the internal list 
	ModuleTable[name]=table;
	// register the mobule table with python
	Py_InitModule(name.c_str(),table);
	// update the include file
	createPythonIncludeScript();
	return true;
}

/*! return the one and only instance of the class
 *  this static function is part of the singleton implementation. If you need thread
 *  safe access to the interpreter, you have to protect this call e.g. by critial
 *  sections. Anyhow, thread safe functions are difficult to implement in a platform
 *  neutral manner.
 */
CPythonInterface& CPythonInterface::getInstance()
{
	static CPythonInterface py;
	return py;
}

//! reset python interpreter and reinit the interpreter in a similar way as it is 
//! done in the constructur.
bool CPythonInterface::reset()
{
	// clean up; empty the module table, clear all errors and close the interpreter
	ModuleTable.clear();
	PyErr_Clear();
	//Py_Finalize();
	Py_EndInterpreter(mainThreadState);
	//reset Thread
	
	// reinit the interpreter
	mainThreadState = Py_NewInterpreter();
	//Py_Initialize();
	Py_SetProgramName(argv[0]);
	PySys_SetArgvEx(1,argv, 0);
	// add the call-back function both for tracing and profiling
	if (bEnableTrace)
		PyEval_SetTrace(TraceCallback, NULL);
	if (bEnableProfile)
		PyEval_SetProfile(TraceCallback, NULL);

	ASSERT(Py_IsInitialized());
	
	initPythonInterface();
	
	createPythonIncludeScript();
	wcInitPythonBinding();
	return true;
}
