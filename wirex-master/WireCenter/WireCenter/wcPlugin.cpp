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

#include "StdAfx.h"
#include "wcPlugin.h"
#include <WireLib/WireLib.h>

// initialize the base ID
const int CwcPlugin::baseID=40000;

CwcPlugin::CwcPlugin(const std::string& Filename)
: filename(Filename), handle(0), Init(0), Invoke(0), PyInvoke(0)
{
	App.pGUI=0;
	App.pPyInterface=0;
	App.pRobotDoc=0;
	App.pScene=0;
	App.wcVersion=PCRL::versionString;
}

CwcPlugin::~CwcPlugin() 
{ 
	release(); 
}

//! release the DLL, free the handle
bool CwcPlugin::release()
{
	if (handle)
		if (!FreeLibrary(handle))
		{
			printf("ERROR: Failed to release %s\n",filename.c_str());	
			return false;
		}
	// reset the internal pointer after unloading the dll
	handle=0;
	Init=0;
	Invoke=0;
	PyInvoke=0;
	return true;
}

/*! tries to dynamically load the DLL specified by filename.
 *  after loading the DLL, the function pointer for Init and Invoke are
 *  established.
 *  \return false, if the library cannot be loaded or if the interface functions
 * 		could not be found.
 */
bool CwcPlugin::load()
{
	// load the plug-in library at runtime
	// we have a constant name here but if could be selected from a file dialog as well
	handle=LoadLibrary(filename.c_str());
	if (!handle)
	{
		printf("ERROR: Loading %s.dll failed\n",filename.c_str());
		return false;
	}
	// try to get the handler for the init function
	Init = (PWCINITPLUGIN)GetProcAddress(handle,"wcInitPlugin");
	if (!Init)
	{
		printf("ERROR: Could not find wcInitPlugin\n");
		release();
		return false;
	}
	else
		// we have the function pointer to the init fuction; thus we call it
		Init(&App,&Plugin);

	// try to get a handler for the invoke function
	Invoke = (PWCINVOKE)GetProcAddress(handle,"wcInvoke");
	if (!Invoke)
	{
		printf("ERROR: Could not find wcInvoke in the DLL\n");
		release();
		return false;
	}

	// try to get a handler for the PyInvoke function
	PyInvoke = (PWCPYINVOKE)GetProcAddress(handle,"pyInvoke");
	if (!PyInvoke)
	{
		printf("ERROR: Could not find pyInvoke in the DLL\n");
		release();
		return false;
	}

	return true;
}

//! print the plugin name and its exported functions
bool CwcPlugin::printInfos()
{
	if (!handle)
		return false;
	// print infos and function that we loaded from the dll
	printf("Name: %s\nDesc: %s\nVersion: %s\n",Plugin.name.c_str(),Plugin.desc.c_str(),Plugin.version.c_str());
	printf("Number of function: %i\n",Plugin.methodCount);
	printf("The following functions are provided:\n");
	wcMethodTable* pTable = (wcMethodTable*)Plugin.methodTable;
	while (pTable->attributes!=0)
	{
		printf("%s : %s %s\n",pTable->name,pTable->desc,pTable->help);
		pTable++;
	}
	printf("\n");
	return true;
}

//! add new entries to the menu 
//! this function is deprecated and has to be replaced by a new implementation
//! that deals with the ribbon menus
bool CwcPlugin::extendMenu()
{
	// tinker around with the main menu bar of wire center
	CMenu* pTopMenu = AfxGetMainWnd()->GetMenu();
	// check if we received a pointer to the top level menu
	if (!pTopMenu)
		return false;

	// get the second last menu item (it should be "Script"
	CMenu* pSub = pTopMenu->GetSubMenu(pTopMenu->GetMenuItemCount()-2);

	pSub->AppendMenuA(MF_SEPARATOR);
	if (pSub)
	{
		// loop through the method table of the module and add one item to the menu for each command
		wcMethodTable* pTable = (wcMethodTable*)Plugin.methodTable;
		for (int id=0; id<Plugin.methodCount; id++)
		{
			CString str;
			str.Format("%s::%s",Plugin.pyModuleName.c_str(),pTable[id].name);
			pSub->AppendMenuA(MF_STRING|MF_ENABLED,baseID+id+1,str);
		}
	}
	return true;
}

//! create the method table to be used by the python interpreter
bool CwcPlugin::getPyMethodTable(PyMethodDef*& dllMethodTable, std::string& moduleName)
{
	moduleName = Plugin.pyModuleName;
	dllMethodTable = new PyMethodDef[Plugin.methodCount+1];
	wcMethodTable* pTable = (wcMethodTable*)Plugin.methodTable;
	int count=0;
	while (pTable->attributes!=0)
	{
		dllMethodTable[count].ml_name = pTable->name;
		dllMethodTable[count].ml_meth = (PyCFunction)GetProcAddress(handle,pTable->name);
		if (!dllMethodTable[count].ml_meth)
			printf("no function pointer for %s\n",dllMethodTable[count].ml_name);
		else
			printf("python function %s.%s loaded into table\n",moduleName.c_str(),dllMethodTable[count].ml_name);
		dllMethodTable[count].ml_flags = METH_NOARGS;
		dllMethodTable[count].ml_doc = pTable->help;

		count++;	
		pTable++;
	}
	dllMethodTable[count].ml_name = NULL;
	dllMethodTable[count].ml_meth = 0;
	dllMethodTable[count].ml_flags = NULL;
	dllMethodTable[count].ml_doc = NULL;

	return true;
}
