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

/*! \file wcPyBindings.h
 *
 *	\author   Andreas Pott
 *
 *  \dependency
 *		Python      for scripting engine (strong dependency)
 *		WireLib		interfacing with robot and workspace API
 *      WireCenter	interfacing with the GUI
 *
 *  \brief 
 *		This file provides the python binding inside the application.
 *		Through the method tables provided for objects of type CPythonInterface 
 *      one can call internal methods of the applications object and the robot. 
 *		The python interface allows to access more function than the GUI, 
 *		especially it is possible to change parameters where no GUI element is 
 *		available.
 *
 *	\remark
 *		The wcPyBindings.h header itself does not introduce additional dependency
 *		and also works as decoupling element between the application layer and the 
 *		libraray. Since WireCenter allows also for sciprint its GUI behaviour this
 *		decoupling cannot be complete.
 */

#pragma once

//! init the python scripting table for WireCenter and WireLib's function binding
bool wcInitPythonBinding();
