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

/*! \file WiPy.cpp
 *
 *	\author   Philipp Miermeister
 *
 *  \dependency
 *		Python		scripting interface
 *		WireLib		function library to be interfaced
 *      WireLibPyBindings
 *					the method table with adapter functions converting function calls in
 *                  python to wirelib
 *
 *  \brief
 *  WiPy is the main file to build a stand-alone python interface library called WiPy
 *  allowing to use parts of wirelib in python.
 */

#include "pyBindings.h"
#include <WireLibPyBindings/WireLibPyBindings.h>

using namespace PCRL;

// important: add all python moudles that should be export under Properties->Linker->Command Line->Additional Options
// as /export:initWiPy

/*! \todo it seems that we can use the function Py_initModule3(...) to pass as third parameter a docstring to
 *  be connected to each namespace
 */
static struct PyModuleDef WiPyModule = {
	PyModuleDef_HEAD_INIT,
	"WiPy",
	"WiPy (WireLib Python) is a stand-alone python module for analysis and synthesis "
	"of cable-driven parallel robots. It provides the scripting API that is used inside WireCenter as a stand alone "
	"python module and it can be used indepedently from the GUI. The API is provided through four modules Irobot, "
	"Ikin, Iws, and Icontrol. Additionally, the WiPy module itself provides some functions for the management of the "
	"extension.\nCopyright 2006-2018 Andreas Pott",
	-1,
	WireLibMethods};

static struct PyModuleDef IrobotModule = {
	PyModuleDef_HEAD_INIT,
	"Irobot",
	"Irobot allows to create and modify robot objects such as manipulation "
	"of geometry, loading, saving, configuration of technical parameters",
	-1,
	EmbMethodsIrobot};

static struct PyModuleDef IkinModule = {
	PyModuleDef_HEAD_INIT,
	"Ikin",
	"Ikin is a collection of function to perform computations related to robot "
	"kinematics and statics",
	-1,
	EmbMethodsIkin};

static struct PyModuleDef IwsModule = {
	PyModuleDef_HEAD_INIT,
	"Iws",
	"Iws provides a collection of method to compute and evaluate the workspace",
	-1,
	EmbMethodsIws};

static struct PyModuleDef IcontrolModule = {
	PyModuleDef_HEAD_INIT,
	"Icontrol",
	"Icontrol provides a collection of methods to control the robot",
	-1,
	EmbMethodsControl};

PyMODINIT_FUNC initWiPy(void)
{
	// add and export the module exported by WireLibPyBindings to the C-extensions module table

	PyObject *pWiPy = PyModule_Create(&WiPyModule);
	PyObject *pIrobot = PyModule_Create(&IrobotModule);
	PyObject *pIkin = PyModule_Create(&IkinModule);
	PyObject *pIws = PyModule_Create(&IwsModule);
	PyObject *pIcontrol = PyModule_Create(&IcontrolModule);

	// the library WiPy performs some automatic initialization after generation: Import the namespaces and create a new robot
	// this additional imports are propably required to maintain compatibility with old script. the new submodule technique
	// requires additional imports are prefacing such as WiPy.Irobot.methodName()
	PyRun_SimpleString("import Irobot");
	PyRun_SimpleString("import Ikin");
	PyRun_SimpleString("import Iws");
	PyRun_SimpleString("import Icontrol");

	// add the four extension modules to the WiPy core modul
	Py_INCREF(pIcontrol);
	Py_INCREF(pIws);
	Py_INCREF(pIkin);
	Py_INCREF(pIrobot);

	PyModule_AddObject(pWiPy, "Irobot", pIrobot);
	PyModule_AddObject(pWiPy, "Iws", pIws);
	PyModule_AddObject(pWiPy, "Ikin", pIkin);
	PyModule_AddObject(pWiPy, "Icontrol", pIcontrol);

	// add a variable to the interface; it seems that the value of the variable cannot be
	// permanently linked to something in WiPy
	PyObject *var = Py_BuildValue("d", 42.0);
	PyModule_AddObject(pWiPy, "life", var);

	// generate the default robot
	createRobotCore();
}

// Define module initialization function for Python 3
PyMODINIT_FUNC PyInit_WiPy(void)
{
	// Create main module
	PyObject *pWiPy = PyModule_Create(&WiPyModule);
	if (!pWiPy)
		return NULL;

	// Create submodules and add them to the main module
	PyObject *pIrobot = PyModule_Create(&IrobotModule);
	PyObject *pIkin = PyModule_Create(&IkinModule);
	PyObject *pIws = PyModule_Create(&IwsModule);
	PyObject *pIcontrol = PyModule_Create(&IcontrolModule);

	if (pIrobot)
	{
		Py_INCREF(pIrobot);
		PyModule_AddObject(pWiPy, "Irobot", pIrobot);
	}
	if (pIkin)
	{
		Py_INCREF(pIkin);
		PyModule_AddObject(pWiPy, "Ikin", pIkin);
	}
	if (pIws)
	{
		Py_INCREF(pIws);
		PyModule_AddObject(pWiPy, "Iws", pIws);
	}
	if (pIcontrol)
	{
		Py_INCREF(pIcontrol);
		PyModule_AddObject(pWiPy, "Icontrol", pIcontrol);
	}

	// Add any other constants or initialization code needed
	PyObject *var = Py_BuildValue("d", 42.0);
	PyModule_AddObject(pWiPy, "life", var);

	// Optional: Initialize robot or other components
	createRobotCore(); // Ensure this function is compatible with your application logic

	return pWiPy;
}

int main(int argc, char *argv[])
{
	/// Convert argv[0] to a wide character string
	wchar_t *programName = Py_DecodeLocale(argv[0], NULL);
	if (!programName)
	{
		fprintf(stderr, "Fatal error: cannot decode argv[0]\n");
		exit(1);
	}

	// Set the name of the program as Python expects a wide string
	Py_SetProgramName(programName);

	/* Initialize the Python interpreter.  Required. */
	Py_Initialize();

	/* Add a static module */
	PyInit_WiPy();
}

/*
Old Code
PyObject* pWiPy =
		Py_InitModule3("WiPy", WireLibMethods, "WiPy (WireLib Python) is a stand-alone python module for analysis and synthesis "
		"of cable-driven parallel robots. It provides the scripting API that is used inside WireCenter as a stand alone "
		"python module and it can be used indepedently from the GUI. The API is provided through four modules Irobot, "
		"Ikin, Iws, and Icontrol. Additionally, the WiPy module itself provides some functions for the management of the "
		"extension.\nCopyright 2006-2018 Andreas Pott");
	PyObject* pIrobot =
		Py_InitModule3("Irobot", EmbMethodsIrobot, "Irobot allows to create and modify robot objects such as manipulation "
		"of geometry, loading, saving, configuration of technical parameters");
	PyObject* pIkin =
		Py_InitModule3("Ikin", EmbMethodsIkin, "Ikin is a collection of function to perform computations related to robot "
		"kinematics and statics");
	PyObject* pIws =
		Py_InitModule3("Iws", EmbMethodsIws, "Iws provides a collection of method to compute and evaluate the workspace");
	PyObject* pIcontrol =
		Py_InitModule3("Icontrol", EmbMethodsControl, "Icontrol has functions to load and process path data loaded from nc files.");

*/