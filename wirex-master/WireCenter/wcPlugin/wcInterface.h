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

/*! \file wcInterface.h                                                   
 *
 *	\author   Andreas Pott
 *
 *  \dependency
 *		WireCenter		provide the application layer objects to be interfaced with
 *
 *  Declarations of data structures used for interface extension DLLs (plug-ins) for WireCenter
 */

#pragma once

#include<string>

/*! This structure is passed by the host poccess to the plug-in dll to provide 
 *  it with data from the application object. Thus, the IwcApp data strcuture 
 *  is normally filled by wireCenter to provide details of the running instance
 *  for the plugin, e.g. to access the current RobotDocument.
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
  * containts data of the plugin to be interpreted by the running instance of WireCenter.
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
	// more data might come
};
