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

/*! \file wcPlugin.h
 *
 *	\author   Andreas Pott
 *
 *  \brief This class creates and manages the interface to a DLL
 *  which is called a plugin for WireCenter. The DLL provides additional
 *  functions and/or dependencies to third party software. Decoupling 
 *  extensions through plugins does not introduces these dependencies
 *  into the core code of WireCenter. Thus, one can implement optional extensions
 *  that need software components that might be not available on every
 *  computer. But such as extension may be deployed as binary to other computer.
 */

#pragma once

#include <string>
#include <WireCenter/wcPlugin/wcDllApi.h>

class CwcPlugin
{
public:
	std::string filename;		//!< filename of the dll to be loaded
	HINSTANCE handle;			//!< handle to the instance of the loaded dll
	IwcApp App;					//!< data structure to be filled by wirecenter to provide the DLL information about the state of wirecenter
	IwcPlugin Plugin;			//!< data structure filled by the plugin: containts the method tables of the plugin
	PWCINITPLUGIN Init;			//!< function pointer to the init function inside the dll
	PWCINVOKE Invoke;			//!< function pointer to the Invoke function. used to access the method table inside the dll
	PWCPYINVOKE PyInvoke;		//!< function pointer to the Python Invoke function. used to access the method table inside the dll
	static const int baseID;	//!< base UID for the menu items to be identified by MFC/WireCenter's message handlers

public:
	CwcPlugin(const std::string& Filename="");
	~CwcPlugin();
	
	//! tries to dynamically load the DLL specified by filename
	bool load();
	//! print the status and content of the loaded DLL
	bool printInfos();
	//! add new entries to the menu bar
	bool extendMenu();
	//! release the DLL
	bool release();
	//! create the method table to be used by the python interpreter
	bool getPyMethodTable(PyMethodDef*& dllMethodTable, std::string& moduleName);
};
