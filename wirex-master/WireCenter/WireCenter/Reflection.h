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

/*! \file		Reflection.h
 *
 *  \author		Andreas Pott
 *
 *  \version	1.1
 *  \date		06.04.2005
 *  \change		22.12.2009
 * 
 *  \class CReflection
 *
 *  \brief Provide a simple reflection mechanism to allow for
 *  Parameter persistance for derived objects and inspection based on symbolic names
 * 
 *  \par Desciption
 *  CReflection provides access to class member through symbolic names.
 *  The implementation allows for parameter persistance for derived classes. 
 *  Derive your class from MoParameterFile to enable the class to
 *  read and write a configuration file. Use the makro MOPARAMBIND in 
 *  the constructur of the class to generate a generic binding of the 
 *  variable passed to the makro. 
 *  Calls to WriteFile(...) write an appropriate parameter file to disk
 *  while cass to ReadFile(...) restore the parameters.
 *  If you do like your variable names as names in the configuration file
 *  you can use the different bind(...) function to define another name
 *  for the variable the parameter file.
 * 
 *  \remark
 *  The implementation is based on the earlier version used in the MoSynthesizer
 *  package. This extended version was motivated by the usage of Python as scripting
 *  language. Thus, the new version is design to support getter/setter function 
 *  that use symbolic names.
 *  
 *  \todo Implement an error handling for file IO.
 */////////////////////////////////////////////////////////////////////

#if !defined(WIRECENTER_REFLECTION)
#define WIRECENTER_REFLECTION

/*
#if _MSC_VER > 1000
#pragma warning(disable:4786)		// Warnung für zu lange Symbole unterdrücken
#endif // _MSC_VER > 1000
*/

#include <map>
#include <string>
#include <iostream>
#include <fstream>

using namespace std;

class CReflection  
{
    static char comment[];
    map<string, bool*> boolMap;
    map<string, int*> intMap;
    map<string, double*> doubleMap;
    map<string, string*> stringMap;

public:
	//! use the bind functions to map a symbolic name to the corresponding variable
    void bind(bool& x, const string& name)   { boolMap[name]= &x; }
    void bind(int& x, const string& name)    { intMap[name]= &x; }
    void bind(double& x, const string& name) { doubleMap[name]= &x; }
    void bind(string& x, const string& name) { stringMap[name]= &x; }

	//! print the content and the state of the binding table to the console
	bool printBindingTable() const;
	//! read the reflectable parameters from a file
    bool ReadFile(const string& filename);			
	//! write the reflectable parameters into a file
    bool WriteFile(const string& filename) const;
	//! generic getter function for the supported types
	bool get(const string& name, bool& x) const;
	bool get(const string& name, int& x) const;
	bool get(const string& name, double& x) const;
	bool get(const string& name, string& x) const;
	//! generic setter functions for the supported types
	bool set(const string& name, bool& x) const;
	bool set(const string& name, int& x) const;
	bool set(const string& name, double& x) const;
	bool set(const string& name, string& x) const;
};

#endif // !defined(WIRECENTER_REFLECTION)
